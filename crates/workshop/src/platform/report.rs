use super::json::{self, Value};

/// Формулировки окна сохраняют область применения результата штатного отчёта.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Verdict {
    Rejected,
    NotProven,
    ProvenInSimulation,
    NotAnalyzed,
}

impl Verdict {
    pub fn text(self) -> &'static str {
        match self {
            Self::Rejected => "Отклонено · Rejected",
            Self::NotProven => "Не доказано · NotProven",
            Self::ProvenInSimulation => "Доказано в области симуляции · PassedSimulation",
            Self::NotAnalyzed => "Готовность не оценена · NotAnalyzed",
        }
    }

    fn from_assembly(value: &Value) -> Result<Self, String> {
        match text(value, "verdict") {
            Some("Rejected") => Ok(Self::Rejected),
            Some("NotProven") => Ok(Self::NotProven),
            Some("PassedSimulation") => Ok(Self::ProvenInSimulation),
            _ => Err("неизвестный вердикт сборки".into()),
        }
    }
}

pub struct Report {
    root: Value,
    pub raw: String,
}

pub fn text<'a>(value: &'a Value, name: &str) -> Option<&'a str> {
    value.get(name)?.str()
}

impl Report {
    pub fn parse(raw: String) -> Result<Self, String> {
        let root = json::parse(&raw)?;
        let status = text(&root, "status").ok_or("в отчёте отсутствует статус конвейера")?;
        for field in ["outcome", "closed_loop"] {
            if root.get(field).is_some()
                && !matches!(text(&root, field), Some("pass" | "fail" | "not_analyzed"))
            {
                return Err(format!("неизвестный исход конвейера в поле {field}"));
            }
        }
        if root.get("outcome").is_some() && root.get("closed_loop").is_some() {
            return Err("в отчёте два поля исхода задачи".into());
        }
        if let Some(assemblies) = root.get("assemblies") {
            if root.get("version").and_then(Value::u64) != Some(1) || status != "completed" {
                return Err("неподдерживаемая версия или статус отчёта кампании".into());
            }
            let assemblies = assemblies.object().ok_or("неверная таблица сборок")?;
            if assemblies.is_empty() {
                return Err("пустая таблица сборок".into());
            }
            for (id, assembly) in assemblies {
                let verdict = Verdict::from_assembly(assembly)?;
                let readiness = text(assembly, "readiness");
                let consistent = match verdict {
                    Verdict::Rejected => matches!(readiness, None | Some("Rejected")),
                    Verdict::NotProven => matches!(readiness, None | Some("NeedsWork")),
                    Verdict::ProvenInSimulation => complete_evidence(assembly),
                    Verdict::NotAnalyzed => false,
                };
                if !consistent {
                    return Err(format!(
                        "противоречивый вердикт, покрытие или готовность сборки «{id}»"
                    ));
                }
            }
        }
        Ok(Self { root, raw })
    }

    pub fn outcome(&self) -> Option<&str> {
        text(&self.root, "outcome").or_else(|| text(&self.root, "closed_loop"))
    }

    pub(crate) fn journal_fingerprint(&self) -> Option<&str> {
        text(&self.root, "log_fingerprint")
    }

    pub fn readiness(&self, assembly: &str) -> Option<&str> {
        text(self.root.get("assemblies")?.get(assembly)?, "readiness")
    }

    pub fn status(&self) -> &str {
        text(&self.root, "status").unwrap_or("неизвестен")
    }
    pub fn assemblies(&self) -> impl Iterator<Item = &str> {
        self.root
            .get("assemblies")
            .and_then(Value::object)
            .into_iter()
            .flat_map(|map| map.keys().map(String::as_str))
    }
    pub fn verdict(&self, assembly: Option<&str>) -> Verdict {
        if assembly.is_none() && self.outcome() == Some("fail") {
            return Verdict::Rejected;
        }
        assembly
            .and_then(|id| self.root.get("assemblies")?.get(id))
            .and_then(|value| Verdict::from_assembly(value).ok())
            .unwrap_or(Verdict::NotAnalyzed)
    }
    pub fn repeat_key(&self) -> Option<&str> {
        text(&self.root, "repeat")
    }

    pub(crate) fn playback(&self, key: &str) -> Option<(u64, u64, &str, Option<u64>)> {
        let assembly = key.split_once('/')?.0;
        let row = self.root.get("assemblies")?.get(assembly)?.get("runs")?.get(key)?;
        if text(row, "status") != Some("completed") {
            return None;
        }
        let detail = row.get("detail")?;
        let mode = text(detail.get("summary")?.get("key")?, "mode")?;
        Some((
            row.get("input")?.u64()?,
            detail.get("journal")?.u64()?,
            mode,
            detail.get("event_ns").and_then(Value::u64),
        ))
    }

    pub fn worst(&self, assembly: Option<&str>) -> Option<&str> {
        match assembly {
            Some(id) => text(self.root.get("assemblies")?.get(id)?, "worst_run_key"),
            None => text(&self.root, "worst_run_key").or_else(|| text(&self.root, "worst_variant")),
        }
    }
    pub fn worst_time(&self, assembly: Option<&str>) -> Option<u64> {
        let record = match assembly {
            Some(id) => self.root.get("assemblies")?.get(id)?,
            None => &self.root,
        };
        record.get("worst_time_ns")?.u64()
    }

    pub(crate) fn worst_variant(&self) -> Option<&str> {
        text(&self.root, "worst_variant")
    }
    pub fn reasons(&self, assembly: Option<&str>) -> Vec<&str> {
        let record = match assembly {
            Some(id) => self.root.get("assemblies").and_then(|v| v.get(id)),
            None => Some(&self.root),
        };
        let mut reasons = Vec::new();
        if let Some(record) = record {
            collect_reasons(record, &mut reasons);
        }
        reasons
    }
}

fn complete_evidence(assembly: &Value) -> bool {
    let count = |row: &Value, name| row.get(name).and_then(Value::u64);
    let (Some(coverage), Some(runs)) = (
        assembly.get("coverage"),
        assembly.get("runs").and_then(Value::object),
    ) else {
        return false;
    };
    let total = runs.values().try_fold(0_u64, |sum, row| {
        let required = count(row, "required")?;
        (required > 0
            && count(row, "executed") == Some(required)
            && text(row, "status") == Some("completed")
            && text(row, "outcome") == Some("pass"))
        .then(|| sum.checked_add(required))?
    });
    total.is_some_and(|n| {
        n > 0
            && count(coverage, "required") == Some(n)
            && count(coverage, "executed") == Some(n)
            && count(coverage, "open") == Some(0)
            && count(coverage, "omitted") == Some(0)
            && matches!(
                text(assembly, "readiness"),
                Some("NeedsWork" | "BenchOnly" | "Passed")
            )
    })
}

fn collect_reasons<'a>(value: &'a Value, out: &mut Vec<&'a str>) {
    if let Some(values) = value.array() {
        for value in values {
            collect_reasons(value, out);
        }
    }
    if let Some(fields) = value.object() {
        for (key, value) in fields {
            if matches!(
                key.as_str(),
                "reason"
                    | "detail"
                    | "component_failure"
                    | "missing"
                    | "missing_data"
                    | "coverage_gaps"
                    | "safety_gaps"
                    | "diagnostics"
                    | "text"
            ) {
                out.extend(value.str());
                out.extend(value.array().into_iter().flatten().filter_map(Value::str));
            }
            collect_reasons(value, out);
        }
    }
}
