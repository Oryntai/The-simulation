use std::collections::{BTreeMap, BTreeSet};

use serde::{Deserialize, Serialize};
use sim_runtime::control::{self, EventKind};
use sim_runtime::{LogKind, Runtime};
use sim_signal::Time;

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct Reference {
    pub file: String,
    #[serde(default)]
    pub diagnostic_only: bool,
    revision: Option<String>,
    time_column: String,
    interval_ns: u64,
    horizon_ns: u64,
    tolerance: f64,
    columns: Vec<Column>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Column {
    reference: String,
    signal: String,
    offset_ns: u64,
    #[serde(default)]
    absolute: bool,
    #[serde(default)]
    exact: bool,
    normalize_from: Option<f64>,
    packet: Option<PacketField>,
}

/// Два байта little-endian из последнего доставленного пакета нужного формата.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct PacketField {
    route: String,
    length: usize,
    prefix: Vec<u8>,
    bytes_le: [usize; 2],
    initial: u16,
}

#[derive(Serialize, Deserialize)]
pub(super) struct Comparison {
    pub fingerprint: u64,
    pub passed: bool,
    pub errors: Vec<Error>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub journal: Option<u64>,
}

#[derive(Serialize, Deserialize)]
pub(super) struct Error {
    column: String,
    signal: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    unit: Option<String>,
    samples: usize,
    different_samples: usize,
    reference_peak_abs: f64,
    max_abs_error: f64,
    max_relative_error: Option<f64>,
    worst_ns: u64,
    passed: bool,
    #[serde(skip)]
    actual: Vec<(u64, f64)>,
}

impl Error {
    fn add(&mut self, at: u64, actual: f64, expected: f64) -> Result<(), String> {
        let delta = (actual - expected).abs();
        if !delta.is_finite() {
            return Err("неконечная погрешность эталонного отсчёта".into());
        }
        self.samples += 1;
        self.actual.push((at, actual));
        self.different_samples += usize::from(delta != 0.0);
        self.reference_peak_abs = self.reference_peak_abs.max(expected.abs());
        if delta > self.max_abs_error {
            self.max_abs_error = delta;
            self.worst_ns = at;
        }
        Ok(())
    }
}

#[derive(Serialize)]
pub(super) struct ModelDifference {
    column: String,
    samples: usize,
    max_abs: f64,
    relative_to_reference_peak: Option<f64>,
}

impl Comparison {
    pub(super) fn absolute_errors(&self) -> impl Iterator<Item = (&str, &str, f64)> {
        self.errors.iter().filter_map(|error| {
            Some((
                error.column.as_str(),
                error.unit.as_deref()?,
                error.max_abs_error,
            ))
        })
    }

    /// Сравнивает две рассчитанные кривые на одной сетке эталонного опыта.
    pub(super) fn difference(&self, other: &Self) -> Result<Vec<ModelDifference>, String> {
        if self.fingerprint != other.fingerprint || self.errors.len() != other.errors.len() {
            return Err("варианты относятся к разным сеткам или рядам".into());
        }
        self.errors
            .iter()
            .zip(&other.errors)
            .map(|(a, b)| {
                if a.column != b.column
                    || a.actual.is_empty()
                    || a.actual.len() != b.actual.len()
                    || a.reference_peak_abs != b.reference_peak_abs
                {
                    return Err("разный набор отсчётов при сравнении моделей".into());
                }
                let mut max_abs: f64 = 0.0;
                for (&(ta, va), &(tb, vb)) in a.actual.iter().zip(&b.actual) {
                    let error = (va - vb).abs();
                    if ta != tb || !error.is_finite() {
                        return Err("несовместимые отсчёты при сравнении моделей".into());
                    }
                    max_abs = max_abs.max(error);
                }
                let relative = max_abs / a.reference_peak_abs;
                if a.reference_peak_abs != 0.0 && !relative.is_finite() {
                    return Err("переполнение нормировки чувствительности".into());
                }
                Ok(ModelDifference {
                    column: a.column.clone(),
                    samples: a.samples,
                    max_abs,
                    relative_to_reference_peak: (a.reference_peak_abs != 0.0).then_some(relative),
                })
            })
            .collect()
    }
}

impl Reference {
    pub(super) fn absolute_difference(
        &self,
        text: &str,
        runtime: &Runtime,
        unit: &str,
        revision: &str,
    ) -> Result<(f64, u64), String> {
        let [column] = self.columns.as_slice() else {
            return Err("для абсолютной ошибки нужен один сопоставимый ряд".into());
        };
        if self.revision.as_deref() != Some(revision)
            || column.packet.is_some()
            || column.absolute
            || column.normalize_from.is_some()
            || column.offset_ns != 0
            || !runtime
                .log()
                .header()
                .truth
                .iter()
                .any(|spec| spec.name == column.signal && spec.unit == unit)
        {
            return Err("не совпадают версия, сигнал, единица или шкала времени замера".into());
        }
        let comparison = self.compare(text, runtime)?;
        let error = &comparison.errors[0];
        Ok((error.max_abs_error, error.worst_ns))
    }

    pub(super) fn validate(&self) -> Result<(), String> {
        if !self.tolerance.is_finite()
            || self.tolerance < 0.0
            || self.interval_ns == 0
            || self.horizon_ns == 0
            || self.columns.is_empty()
        {
            return Err("некорректный допуск, сетка или горизонт эталона".into());
        }
        let mut names = BTreeSet::new();
        for column in &self.columns {
            if !names.insert(&column.reference) {
                return Err("повтор ряда эталона".into());
            }
            if column.normalize_from.is_some_and(|x| !x.is_finite()) {
                return Err("неконечная точка отсчёта для нормировки".into());
            }
            if let Some(packet) = &column.packet
                && (packet.route.trim().is_empty()
                    || packet.prefix.is_empty()
                    || packet.prefix.len() > packet.length
                    || packet.bytes_le[0] == packet.bytes_le[1]
                    || packet.bytes_le.iter().any(|&i| i >= packet.length))
            {
                return Err("некорректная схема поля пакета для эталона".into());
            }
        }
        Ok(())
    }

    pub(super) fn validate_text(&self, text: &str) -> Result<(), String> {
        self.read_rows(text).map(|_| ())
    }

    fn read_rows(&self, text: &str) -> Result<Vec<(u64, Vec<f64>)>, String> {
        let mut lines = text.lines();
        let columns: Vec<_> = lines.next().ok_or("пустой эталон")?.split(',').collect();
        if columns.iter().copied().collect::<BTreeSet<_>>().len() != columns.len() {
            return Err("повтор столбца в эталоне".into());
        }
        let column_id = |name: &str| {
            columns
                .iter()
                .position(|column| *column == name)
                .ok_or_else(|| format!("в эталоне нет столбца «{name}»"))
        };
        let time_column = column_id(&self.time_column)?;
        let wanted = self
            .columns
            .iter()
            .map(|column| column_id(&column.reference))
            .collect::<Result<Vec<_>, _>>()?;
        let mut rows = Vec::new();
        for line in lines {
            let data = line
                .split(',')
                .map(|cell| {
                    cell.parse::<f64>()
                        .ok()
                        .filter(|v| v.is_finite())
                        .ok_or("ожидается числовой эталон с конечными отсчётами")
                })
                .collect::<Result<Vec<_>, _>>()?;
            if data.len() != columns.len() {
                return Err("длина строки эталона не совпадает с заголовком".into());
            }
            let at = Time::from_secs(data[time_column]).ok_or("неверное время эталона")?;
            if Some(at.0) != (rows.len() as u64).checked_mul(self.interval_ns)
                || at.0 >= self.horizon_ns
            {
                return Err("эталон отклоняется от полной равномерной сетки".into());
            }
            rows.push((at.0, wanted.iter().map(|&i| data[i]).collect::<Vec<_>>()));
        }
        if rows.len() as u64 != self.horizon_ns.div_ceil(self.interval_ns) {
            return Err("эталон покрывает часть временной сетки".into());
        }
        Ok(rows)
    }

    pub(super) fn compare(&self, text: &str, runtime: &Runtime) -> Result<Comparison, String> {
        let log = runtime.log();
        if self.horizon_ns != log.end().time.0 {
            return Err("горизонт эталона отличается от горизонта прогона".into());
        }
        let rows = self.read_rows(text)?;
        let mut errors = Vec::new();
        let mut truth = BTreeMap::<u32, Vec<usize>>::new();
        for (i, column) in self.columns.iter().enumerate() {
            let mut error = Error {
                column: column.reference.clone(),
                signal: column.signal.clone(),
                unit: None,
                samples: 0,
                different_samples: 0,
                reference_peak_abs: 0.0,
                max_abs_error: 0.0,
                max_relative_error: None,
                worst_ns: 0,
                passed: false,
                actual: Vec::with_capacity(rows.len()),
            };
            if let Some(packet) = &column.packet {
                error.unit = Some("1".into());
                let control = control::run(runtime).ok_or("нет живого журнала пакетов")?;
                let mut deliveries = control
                    .events()
                    .iter()
                    .filter_map(|event| {
                        if let EventKind::PacketDelivered { route, bytes, .. } = &event.kind
                            && route == &packet.route
                            && bytes.len() == packet.length
                            && bytes.starts_with(&packet.prefix)
                        {
                            let value = u16::from_le_bytes(packet.bytes_le.map(|i| bytes[i]));
                            Some((event.at.0, f64::from(value)))
                        } else {
                            None
                        }
                    })
                    .peekable();
                if deliveries.peek().is_none() {
                    return Err(format!(
                        "в маршруте «{}» нет доставленных пакетов нужного формата",
                        packet.route
                    ));
                }
                let mut value = f64::from(packet.initial);
                for (at, data) in &rows {
                    let at = at
                        .checked_add(column.offset_ns)
                        .ok_or("смещение вне шкалы")?;
                    if at > self.horizon_ns {
                        return Err("смещение эталона за горизонтом".into());
                    }
                    while let Some((delivered, next)) = deliveries.peek().copied() {
                        if delivered > at {
                            break;
                        }
                        value = next;
                        deliveries.next();
                    }
                    let expected = data[i];
                    if expected != f64::from(expected as u16) {
                        return Err("эталонное поле пакета должно быть целым u16".into());
                    }
                    error.add(at, value, expected)?;
                }
            } else {
                let signal = log
                    .header()
                    .truth
                    .iter()
                    .position(|spec| spec.name == column.signal)
                    .ok_or_else(|| format!("в журнале нет сигнала «{}»", column.signal))?
                    as u32;
                error.unit = Some(log.header().truth[signal as usize].unit.clone());
                truth.entry(signal).or_default().push(i);
            }
            errors.push(error);
        }
        for entry in log
            .entries()
            .iter()
            .filter(|entry| entry.kind == LogKind::Truth)
        {
            let Some(indices) = truth.get(&entry.id) else {
                continue;
            };
            for &i in indices {
                let column = &self.columns[i];
                if errors[i].samples > 0
                    && rows[errors[i].samples - 1].0.checked_add(column.offset_ns)
                        == Some(entry.at.0)
                {
                    return Err(format!(
                        "повтор отсчёта «{}» на сетке эталона",
                        column.signal
                    ));
                }
                let Some((at, data)) = rows.get(errors[i].samples) else {
                    continue;
                };
                let at = at
                    .checked_add(column.offset_ns)
                    .ok_or("смещение вне шкалы")?;
                if entry.at.0 == at {
                    if !entry.valid {
                        return Err("некорректный отсчёт на сетке эталона".into());
                    }
                    let value = if column.absolute {
                        entry.value.abs()
                    } else {
                        entry.value
                    };
                    let origin = column.normalize_from.unwrap_or(0.0);
                    errors[i].add(at, value - origin, data[i] - origin)?;
                }
            }
        }
        for (column, error) in self.columns.iter().zip(&mut errors) {
            if error.samples != rows.len() {
                return Err(format!(
                    "журнал не покрывает сетку эталона «{}»",
                    column.signal
                ));
            }
            let peak = error.reference_peak_abs;
            if peak != 0.0 {
                let relative = error.max_abs_error / peak;
                if !relative.is_finite() {
                    return Err("неконечная относительная погрешность".into());
                }
                error.max_relative_error = Some(relative);
            }
            error.passed = if column.exact || column.packet.is_some() {
                error.different_samples == 0
            } else {
                let allowed = self.tolerance * peak;
                if !allowed.is_finite() {
                    return Err("переполнение допуска эталонного ряда".into());
                }
                error.max_abs_error <= allowed
            };
        }
        Ok(Comparison {
            fingerprint: sim_signal::fnv1a(text.bytes()),
            passed: errors.iter().all(|error| error.passed),
            errors,
            journal: Some(log.fingerprint()),
        })
    }
}
