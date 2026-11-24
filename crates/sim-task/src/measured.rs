//! Односторонняя оценка загруженных измерений с явным происхождением.

use std::collections::BTreeSet;

use serde::Deserialize;

use crate::{
    Error, Fingerprint, Outcome,
    campaign::RunKey,
    invalid,
    verdict::{Finding, HazardClass, Level, LimitValue, RequirementKey},
};

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MeasuredResult {
    version: u32,
    mode: String,
    object: String,
    scope: String,
    origin: Origin,
    check: Check,
}

#[derive(Debug, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
enum Origin {
    Synthetic {
        reference: String,
    },
    Measured {
        report: String,
        revision: String,
        calibration: String,
    },
}

#[derive(Debug, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
enum Check {
    TissueLoad {
        region: String,
        limit_source: String,
        max_force_n: f64,
        max_pressure_pa: f64,
        max_gap_ns: u64,
        samples: Vec<TissueSample>,
    },
    Ingress {
        limit_source: String,
        protected_locations: BTreeSet<String>,
        samples: Vec<IngressSample>,
    },
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Reading {
    value: f64,
    uncertainty: f64,
}

impl Reading {
    fn bounds(&self) -> Result<[f64; 2], Error> {
        let upper = self.value + self.uncertainty;
        if self.value < 0.0 || self.uncertainty < 0.0 || !upper.is_finite() {
            return Err(invalid("некорректное значение или погрешность измерения"));
        }
        Ok([(self.value - self.uncertainty).max(0.0), upper])
    }
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct TissueSample {
    at_ns: u64,
    force_n: Reading,
    area_m2: Reading,
    pressure_map: Vec<PressureCell>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct PressureCell {
    location: String,
    area_m2: f64,
    pressure_pa: Reading,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct IngressSample {
    at_ns: u64,
    locations: Vec<IngressLocation>,
    insulation: Insulation,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct IngressLocation {
    location: String,
    volume_m3: Reading,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
enum Insulation {
    Intact,
    Failed,
    Unknown,
}

fn named(s: &str) -> bool {
    !s.trim().is_empty() && !s.chars().any(char::is_control)
}

fn window(
    mut times: impl Iterator<Item = u64>,
    [start, end]: [u64; 2],
    max_gap_ns: u64,
) -> Result<(), Error> {
    if start >= end || times.next() != Some(start) {
        return Err(invalid("ряд не покрывает начало контрольного окна"));
    }
    let mut prev = start;
    for time in times {
        if time <= prev || time - prev > max_gap_ns {
            return Err(invalid("некорректный порядок или пропуск отсчётов"));
        }
        prev = time;
    }
    if prev != end {
        return Err(invalid("ряд не покрывает конец контрольного окна"));
    }
    Ok(())
}

impl TissueSample {
    fn evaluate(&self) -> Result<([f64; 2], [f64; 2]), Error> {
        let force = self.force_n.bounds()?;
        let area = self.area_m2.bounds()?;
        let mut locations = BTreeSet::new();
        let mut mapped_area = 0.0;
        let mut integral = [0.0; 2];
        let mut peak = [0.0_f64; 2];
        for cell in &self.pressure_map {
            if !named(&cell.location)
                || !locations.insert(&cell.location)
                || !cell.area_m2.is_finite()
                || cell.area_m2 <= 0.0
            {
                return Err(invalid("некорректная ячейка карты давления"));
            }
            let pressure = cell.pressure_pa.bounds()?;
            mapped_area += cell.area_m2;
            for i in 0..2 {
                integral[i] += cell.area_m2 * pressure[i];
                peak[i] = peak[i].max(pressure[i]);
            }
        }
        if !mapped_area.is_finite()
            || !integral[1].is_finite()
            || !(area[0]..=area[1]).contains(&mapped_area)
            || force[0] > integral[1]
            || integral[0] > force[1]
        {
            return Err(invalid(
                "площадь или сила не согласуется с интегралом карты давления",
            ));
        }
        Ok((force, peak))
    }
}

#[derive(Default)]
struct Peak {
    bounds: [f64; 2],
    at: u64,
    location: Option<String>,
}

impl Peak {
    fn include(&mut self, [lo, hi]: [f64; 2], at: u64, location: Option<&str>) {
        if lo > self.bounds[0] {
            self.bounds[0] = lo;
            self.at = at;
            self.location = location.map(str::to_owned);
        }
        self.bounds[1] = self.bounds[1].max(hi);
    }

    fn finding(&self, base: &Finding, code: &str, unit: &str, max: f64, reason: &str) -> Finding {
        let mut finding = base.clone();
        finding.code = code.into();
        finding.unit = Some(unit.into());
        finding.measurement = Some(self.bounds);
        finding.limit = Some(LimitValue::upper([max; 2]));
        if self.bounds[0] > max {
            finding.outcome = Outcome::Fail;
            finding.confirmed_hazard = true;
            finding.event_ns = Some(self.at);
            finding.reason = self.location.as_ref().map_or_else(
                || reason.into(),
                |location| format!("{reason} в зоне «{location}»"),
            );
            finding.missing_data.clear();
        }
        finding
    }
}

impl MeasuredResult {
    /// Оценивает ряд в окне режима. Отпечаток берётся из исходного артефакта измерения.
    /// Положительный исход требует внешней проверки идентичности, применимости и покрытия.
    pub fn assess(
        &self,
        run: &RunKey,
        window_ns: [u64; 2],
        journal: Fingerprint,
    ) -> Result<Vec<Finding>, Error> {
        if self.version != 1 || self.mode != run.mode || !named(&self.object) || !named(&self.scope)
        {
            return Err(invalid(
                "неверная версия, режим или область измеренного результата",
            ));
        }
        let (class, limit_source) = match &self.check {
            Check::TissueLoad { limit_source, .. } => (HazardClass::TissueLoad, limit_source),
            Check::Ingress { limit_source, .. } => (HazardClass::Ingress, limit_source),
        };
        if !named(limit_source) {
            return Err(invalid("у предела отсутствует источник"));
        }
        let key = RequirementKey {
            class,
            object: self.object.clone(),
            extension: None,
            run: Some(run.clone()),
            window_ns: Some(window_ns),
            level: Level::Simulation,
        };
        let mut base = Finding::open(&key, "ряд не подтверждает превышение применимого предела");
        base.missing_data = vec!["для положительного вывода нужны подтверждённые данные и область применения для этой сборки".into()];
        base.limit_sources.push(limit_source.clone());
        base.model_scope = Some(self.scope.clone());
        base.journal = Some(journal);
        base.measurement_sources = match &self.origin {
            Origin::Synthetic { reference } if named(reference) => {
                vec![format!("Синтетический ряд: {reference}")]
            }
            Origin::Measured {
                report,
                revision,
                calibration,
            } if [report, revision, calibration]
                .into_iter()
                .all(|value| named(value)) =>
            {
                vec![
                    format!("Протокол: {report}"),
                    format!("Версия сборки: {revision}"),
                    format!("Калибровка: {calibration}"),
                ]
            }
            _ => return Err(invalid("неполное происхождение измерения")),
        };
        match &self.check {
            Check::TissueLoad {
                region,
                max_force_n,
                max_pressure_pa,
                max_gap_ns,
                samples,
                ..
            } => {
                if !named(region)
                    || *max_gap_ns == 0
                    || [*max_force_n, *max_pressure_pa]
                        .into_iter()
                        .any(|max| !max.is_finite() || max <= 0.0)
                {
                    return Err(invalid("некорректные пределы тканевой нагрузки"));
                }
                window(samples.iter().map(|s| s.at_ns), window_ns, *max_gap_ns)?;
                let mut force = Peak {
                    at: window_ns[0],
                    ..Peak::default()
                };
                let mut pressure = Peak {
                    at: window_ns[0],
                    ..Peak::default()
                };
                for sample in samples {
                    let (f, p) = sample.evaluate()?;
                    force.include(f, sample.at_ns, Some(region));
                    pressure.include(p, sample.at_ns, Some(region));
                }
                Ok(vec![
                    force.finding(
                        &base,
                        "пик контактной силы",
                        "N",
                        *max_force_n,
                        "ряд силы превышает предел",
                    ),
                    pressure.finding(
                        &base,
                        "локальный пик давления",
                        "Pa",
                        *max_pressure_pa,
                        "ряд карты давления превышает локальный предел",
                    ),
                ])
            }
            Check::Ingress {
                protected_locations,
                samples,
                ..
            } => {
                if protected_locations.is_empty() || !protected_locations.iter().all(|s| named(s)) {
                    return Err(invalid("не заданы контролируемые зоны изоляции"));
                }
                // Для обследования после воздействия существенны границы окна, а не частота опроса.
                window(
                    samples.iter().map(|s| s.at_ns),
                    window_ns,
                    window_ns[1].saturating_sub(window_ns[0]),
                )?;
                if samples[0].insulation != Insulation::Intact {
                    return Err(invalid("нет исправного исходного состояния изоляции"));
                }
                let mut ingress = Peak {
                    at: window_ns[0],
                    ..Peak::default()
                };
                let mut insulation = Peak {
                    at: window_ns[0],
                    ..Peak::default()
                };
                for sample in samples {
                    let mut locations = BTreeSet::new();
                    for location in &sample.locations {
                        if !named(&location.location)
                            || !locations.insert(location.location.as_str())
                        {
                            return Err(invalid("неоднозначная зона измерения проникновения"));
                        }
                        let volume = location.volume_m3.bounds()?;
                        if protected_locations.contains(&location.location) {
                            if sample.at_ns == window_ns[0] && volume[0] > 0.0 {
                                return Err(invalid("защищаемая зона уже мокрая до воздействия"));
                            }
                            ingress.include(volume, sample.at_ns, Some(&location.location));
                        }
                    }
                    if protected_locations
                        .iter()
                        .any(|location| !locations.contains(location.as_str()))
                    {
                        return Err(invalid("отсутствует замер объёма в защищаемой зоне"));
                    }
                    let state = match sample.insulation {
                        Insulation::Intact => [0.0, 0.0],
                        Insulation::Failed => [1.0, 1.0],
                        Insulation::Unknown => [0.0, 1.0],
                    };
                    insulation.include(state, sample.at_ns, None);
                }
                // Нуль задаёт запрет попадания в выделенную зону либо отказа изоляции.
                Ok(vec![
                    ingress.finding(
                        &base,
                        "объём в защищаемой зоне",
                        "m³",
                        0.0,
                        "ряд подтверждает проникновение",
                    ),
                    insulation.finding(
                        &base,
                        "отказ изоляции",
                        "1",
                        0.0,
                        "ряд подтверждает потерю изоляции",
                    ),
                ])
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assess(result: &MeasuredResult) -> Result<Vec<Finding>, Error> {
        let run = RunKey {
            station: "блок".into(),
            mode: result.mode.clone(),
            path: "замер".into(),
            faults: Vec::new(),
            sample: crate::sampling::SampleKey::Nominal,
            grid: "профиль".into(),
        };
        result.assess(&run, [0, 4_000_000], Fingerprint::of("ряд".as_bytes()))
    }

    #[test]
    fn карта_контролирует_площадь_силу_время_и_конечность() {
        let raw = include_str!("../../../devices/stage6/corpus/screen/tissue-load-row.toml");
        let rejected = |modify: fn(&mut Vec<TissueSample>)| {
            let mut data: MeasuredResult = toml::from_str(raw).unwrap();
            let Check::TissueLoad { samples, .. } = &mut data.check else {
                panic!("ожидался ряд давления");
            };
            modify(samples);
            assert!(assess(&data).is_err());
        };
        rejected(|samples| samples[1].force_n.value = f64::NAN);
        rejected(|samples| samples[1].pressure_map[0].pressure_pa.uncertainty = f64::INFINITY);
        rejected(|samples| samples[1].area_m2.value *= 2.0);
        rejected(|samples| samples[1].force_n.value *= 2.0);
        rejected(|samples| {
            samples.remove(1);
        });
    }

    #[test]
    fn известная_погрешность_оставляет_пересечение_с_пределом_открытым() {
        let mut data: MeasuredResult = toml::from_str(include_str!(
            "../../../devices/stage6/corpus/screen/tissue-load-row.toml"
        ))
        .unwrap();
        let Check::TissueLoad {
            max_pressure_pa,
            samples,
            ..
        } = &mut data.check
        else {
            panic!("ожидался ряд давления");
        };
        let hot = &mut samples[1].pressure_map[1].pressure_pa;
        hot.uncertainty = hot.value - *max_pressure_pa;
        let findings = assess(&data).unwrap();
        assert_eq!(findings[1].measurement.unwrap()[0], 80000.0);
        assert!(
            findings
                .iter()
                .all(|f| f.outcome == Outcome::NotAnalyzed && !f.confirmed_hazard)
        );
    }

    #[test]
    fn отказ_изоляции_отклоняет_при_нулевом_объёме_в_защищаемой_зоне() {
        let mut data: MeasuredResult = toml::from_str(include_str!(
            "../../../devices/stage6/corpus/screen/ingress-pair-row.toml"
        ))
        .unwrap();
        let Check::Ingress { samples, .. } = &mut data.check else {
            panic!("ожидался ряд проникновения");
        };
        samples[1].insulation = Insulation::Failed;
        let findings = assess(&data).unwrap();
        assert_eq!(findings[0].outcome, Outcome::NotAnalyzed);
        assert_eq!(findings[1].outcome, Outcome::Fail);
    }
}
