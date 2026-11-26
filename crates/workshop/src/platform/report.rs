use std::collections::BTreeMap;

use sim_cli::view::CampaignBinding;

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
    bindings: BTreeMap<String, CampaignBinding>,
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
            if root.get("historical_open_rows").is_some_and(|history| {
                history != &Value::Null && history.get("rows").and_then(Value::array).is_none()
            }) {
                return Err("неверный реестр открытых строк исходной матрицы".into());
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
        let bindings = if root.get("assemblies").is_some() {
            sim_cli::view::campaign_bindings(&raw)?
        } else {
            BTreeMap::new()
        };
        Ok(Self { root, raw, bindings })
    }

    pub fn outcome(&self) -> Option<&str> {
        text(&self.root, "outcome").or_else(|| text(&self.root, "closed_loop"))
    }

    pub(crate) fn journal_fingerprint(&self) -> Option<&str> {
        text(&self.root, "log_fingerprint")
    }

    pub fn readiness(&self, assembly: &str) -> Option<&str> {
        let row = self.root.get("assemblies")?.get(assembly)?;
        if text(row, "verdict") == Some("PassedSimulation")
            && self.verdict(Some(assembly)) == Verdict::NotProven
        {
            return Some("NeedsWork");
        }
        text(row, "readiness")
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
        let Some((id, row)) =
            assembly.and_then(|id| Some((id, self.root.get("assemblies")?.get(id)?)))
        else {
            return Verdict::NotAnalyzed;
        };
        match Verdict::from_assembly(row) {
            Ok(Verdict::ProvenInSimulation)
                if self.registry_notice().is_some()
                    || depth_coverage_open(row)
                    || self.historical_rows(Some(id)).next().is_some() =>
            {
                Verdict::NotProven
            }
            Ok(verdict) => verdict,
            Err(_) => Verdict::NotAnalyzed,
        }
    }

    pub(crate) fn registry_notice(&self) -> Option<&'static str> {
        self.root.get("assemblies")?;
        match self.root.get("registry_version").and_then(Value::u64) {
            Some(3) => None, // Версия реестра классов ТЗ-3.
            Some(2) => Some("реестр v2, без глубинных классов — не доказательство по классам ТЗ-3"),
            _ => Some("версия реестра не подтверждает покрытие классов ТЗ-3"),
        }
    }

    fn historical_rows<'a>(&'a self, assembly: Option<&str>) -> impl Iterator<Item = &'a Value> {
        self.root
            .get("historical_open_rows")
            .and_then(|history| history.get("rows"))
            .and_then(Value::array)
            .into_iter()
            .flatten()
            .filter(move |row| {
                let owner = row
                    .get("key")
                    .and_then(|key| text(key, "station"))
                    .and_then(|station| station.split_once('/'))
                    .map(|(owner, _)| owner);
                assembly.is_none_or(|id| owner.is_none_or(|owner| owner == id))
            })
    }

    pub fn repeat_key(&self) -> Option<&str> {
        text(&self.root, "repeat")
    }

    pub fn playback(&self, key: &str) -> Option<&CampaignBinding> {
        self.bindings.get(key)
    }

    pub fn view_unavailable(&self) -> Option<&str> {
        text(&self.root, "view_unavailable")
    }

    pub fn worst(&self, assembly: Option<&str>) -> Option<&str> {
        match assembly {
            Some(id) => text(self.root.get("assemblies")?.get(id)?, "worst_run_key"),
            None => text(&self.root, "worst_run_key").or_else(|| text(&self.root, "worst_variant")),
        }
    }
    /// Метка воспроизведения выбранной худшей строки или задачи.
    pub fn worst_time(&self, assembly: Option<&str>) -> Option<u64> {
        if self.root.get("assemblies").is_some() {
            return self.playback(self.worst(assembly)?)?.playback.event.as_ref()
                .map(|event| event.at_ns);
        }
        self.root.get("worst_time_ns")?.u64()
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
            if depth_coverage_open(record) {
                reasons.push("глубинное покрытие неполное");
            }
        }
        reasons.extend(self.registry_notice());
        if assembly.is_some() {
            for row in self.historical_rows(assembly) {
                reasons.push(text(row, "reason").unwrap_or("открыта строка исходной матрицы"));
            }
        }
        reasons
    }
}

fn depth_coverage_open(assembly: &Value) -> bool {
    assembly.get("depth_coverage").is_some_and(|coverage| {
        coverage != &Value::Null
            && (text(coverage, "status") != Some("completed")
                || coverage
                    .get("open_rows")
                    .and_then(Value::array)
                    .is_none_or(|rows| !rows.is_empty()))
    })
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
