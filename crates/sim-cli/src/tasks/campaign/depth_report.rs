//! Сводка фактического покрытия. Открытые постановки остаются в знаменателе.

use std::{
    collections::{BTreeMap, BTreeSet},
    path::{Path, PathBuf},
};

use serde::{Deserialize, Serialize};
use sim_task::{
    Fingerprint, Outcome,
    campaign::{
        RunKey,
        depth_factors::{DepthFactors, Selection},
        depth_search::{Margin, Refinement},
    },
};

use super::{RowReport, RunStatus, encode, unavailable_reason};
use crate::{
    tasks::{matrix::Checkpoint, processes},
    view::{CampaignBinding, CampaignPlayback},
};

#[derive(Serialize)]
struct Count {
    required: usize,
    planned: usize,
    executed: usize,
}

#[derive(Serialize)]
pub(super) struct Summary {
    scope: &'static str,
    status: &'static str,
    pairs: Count,
    safety_triples: Count,
    #[serde(skip_serializing_if = "Option::is_none")]
    safety_triples_not_required: Option<String>,
    open_rows: Vec<OpenRow>,
    pub(super) worst: Option<Worst>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub(super) refinement: Option<Refinement>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub(super) acceptance: Option<Acceptance>,
}

#[derive(Serialize)]
pub(super) struct Acceptance {
    pub reserved_runs: u32,
    pub confirmed_launches: u64,
    pub elapsed_ns: u64,
    pub repeat: RepeatStatus,
}

#[derive(Serialize)]
struct OpenRow {
    run_key: String,
    key: Option<RunKey>,
    order: Option<usize>,
    status: RunStatus,
    reason: String,
}

#[derive(Deserialize)]
struct SelectionReceipt {
    selection: Selection,
    key: RunKey,
    input: Fingerprint,
    order: Option<usize>,
}

#[derive(Serialize)]
pub(super) struct Worst {
    pub(super) run_key: String,
    key: RunKey,
    selection: Selection,
    plan_input: Fingerprint,
    input: Fingerprint,
    outcome: Outcome,
    status: RunStatus,
    parameters: Option<BTreeMap<String, f64>>,
    playback: Option<CampaignPlayback>,
    #[serde(skip_serializing_if = "Option::is_none")]
    view_unavailable: Option<String>,
    ranking: &'static str,
    margin: Margin,
}

fn interactions<'a>(
    axes: &[(&'a str, Vec<&'a str>)],
    count: usize,
    selected: &mut Vec<(&'a str, &'a str)>,
    visit: &mut impl FnMut(&[(&'a str, &'a str)]),
) {
    if count == 0 {
        visit(selected);
    } else if axes.len() >= count {
        for (index, &(id, ref levels)) in axes[..=axes.len() - count].iter().enumerate() {
            for &level in levels {
                selected.push((id, level));
                interactions(&axes[index + 1..], count - 1, selected, visit);
                selected.pop();
            }
        }
    }
}

type Axes<'a> = Vec<(&'a str, Vec<&'a str>)>;
type Constraints<'a> = Vec<Vec<(&'a str, &'a str)>>;

/// Для проекции достаточно одного допустимого продолжения. Факторы без
/// ограничений не надо достраивать, а уже несовместимую ветвь можно отсечь.
fn admissible<'a>(
    constrained: &[(&'a str, Vec<&'a str>)],
    rules: &Constraints<'a>,
    selection: &mut BTreeMap<&'a str, &'a str>,
) -> bool {
    if rules.iter().any(|rule| {
        rule.iter()
            .all(|(id, level)| selection.get(id) == Some(level))
    }) {
        return false;
    }
    let Some((index, &(id, ref levels))) = constrained
        .iter()
        .enumerate()
        .find(|(_, (id, _))| !selection.contains_key(id))
    else {
        return true;
    };
    let fits = levels.iter().any(|&level| {
        selection.insert(id, level);
        admissible(&constrained[index + 1..], rules, selection)
    });
    selection.remove(id);
    fits
}

fn count<const N: usize>(
    axes: &Axes<'_>,
    constrained: &Axes<'_>,
    rules: &Constraints<'_>,
    rows: &[(SelectionReceipt, bool)],
) -> Count {
    let mut required = 0;
    interactions(axes, N, &mut Vec::new(), &mut |tuple| {
        required += usize::from(
            rules.is_empty()
                || admissible(constrained, rules, &mut tuple.iter().copied().collect()),
        );
    });
    let mut planned = BTreeSet::new();
    let mut executed = BTreeSet::new();
    for (row, complete) in rows {
        let chosen: Axes<'_> = axes
            .iter()
            .map(|(id, _)| (*id, vec![row.selection[*id].as_str()]))
            .collect();
        interactions(&chosen, N, &mut Vec::new(), &mut |tuple| {
            let projection: [(&str, &str); N] =
                tuple.try_into().expect("размер проекции задан обходом");
            planned.insert(projection);
            if *complete {
                executed.insert(projection);
            }
        });
    }
    Count {
        required,
        planned: planned.len(),
        executed: executed.len(),
    }
}

pub(super) fn summarize(
    factors: &DepthFactors,
    runs: &BTreeMap<String, RowReport>,
    repeat: bool,
    refinement: Option<Refinement>,
) -> Result<Summary, String> {
    let mut rows = Vec::new();
    let mut open_rows = Vec::new();
    let mut worst = None;
    let mut order_seen = BTreeSet::new();
    for (run_key, row) in runs {
        let complete = row.status == RunStatus::Completed && row.executed == 1;
        let annotation = row
            .detail
            .get("coverage")
            .map(|receipt| serde_json::from_value::<SelectionReceipt>(receipt.clone()))
            .transpose()
            .map_err(|e| e.to_string())?;
        if !complete {
            open_rows.push(OpenRow {
                run_key: run_key.clone(),
                key: annotation.as_ref().map(|receipt| receipt.key.clone()),
                order: annotation.as_ref().and_then(|receipt| receipt.order),
                status: row.status,
                reason: unavailable_reason(row),
            });
        }
        let Some(receipt) = annotation else {
            if complete {
                return Err("в выполненной строке покрытия нет квитанции выбранных уровней".into());
            }
            continue;
        };
        if receipt.order.is_none_or(|order| !order_seen.insert(order))
            || receipt.key
                != factors
                    .run_key(&receipt.selection)
                    .map_err(|e| e.to_string())?
            || factors
                .exclusion(&receipt.selection)
                .map_err(|e| e.to_string())?
                .is_some()
            || row.required != 1
            || row.executed != usize::from(row.status == RunStatus::Completed)
            || (complete && row.detail["scenario"]["injections_confirmed"] == false)
            || row
                .playback
                .as_ref()
                .is_some_and(|play| !complete || play.child != receipt.key)
        {
            return Err(format!(
                "несогласованная квитанция строки покрытия «{run_key}»"
            ));
        }
        // Подтверждённое нарушение может произойти до конца опыта. При
        // равном исходе предпочитаем завершённый опыт с доступным журналом.
        if complete || row.outcome == Outcome::Fail {
            let normalized = refinement.as_ref().and_then(|search| search.worst.as_ref())
                .filter(|scored| scored.key == receipt.key && complete && row.playback.is_some());
            let rank = (
                row.outcome,
                complete,
                row.playback.is_some(),
                normalized.is_some(),
                std::cmp::Reverse(receipt.order),
            );
            if worst.as_ref().is_none_or(|(previous, _)| rank > *previous) {
                let installed = row.detail["scenario"]["summary"]
                    .get("installation")
                    .or_else(|| row.detail["scenario"].get("installation"));
                let parameters = installed
                    .and_then(|value| value.get("values"))
                    .map(|value| serde_json::from_value::<BTreeMap<String, f64>>(value.clone()))
                    .transpose()
                    .map_err(|e| e.to_string())?;
                if parameters
                    .iter()
                    .flatten()
                    .any(|(_, value)| !value.is_finite())
                {
                    return Err("неконечное фактически установленное значение".into());
                }
                worst = Some((
                    rank,
                    Worst {
                        run_key: run_key.clone(),
                        key: receipt.key.clone(),
                        selection: receipt.selection.clone(),
                        plan_input: receipt.input,
                        input: row.input,
                        outcome: row.outcome,
                        status: row.status,
                        parameters,
                        playback: row.playback.clone(),
                        view_unavailable: row.playback.is_none().then(|| unavailable_reason(row)),
                        ranking: if normalized.is_some() { "normalized_margin" } else { "outcome" },
                        margin: normalized.map_or(Margin::NotAnalyzed {
                            reason: "в квитанции оценщика отсутствуют привязанный нормированный запас и окно критерия",
                        }, |scored| Margin::Scored { normalized: scored.normalized, outcome: scored.outcome }),
                    },
                ));
            }
        }
        rows.push((receipt, complete));
    }
    open_rows.sort_by_key(|row| (row.order.is_none(), row.order));
    let rules: Constraints<'_> = factors
        .constraints()
        .values()
        .map(|rule| {
            rule.when
                .iter()
                .map(|(id, level)| (id.as_str(), level.as_str()))
                .collect()
        })
        .collect();
    let involved: BTreeSet<_> = rules.iter().flatten().map(|(id, _)| *id).collect();
    let axes: Axes<'_> = factors
        .factors()
        .iter()
        .map(|(id, factor)| (id.as_str(), factor.levels.level_ids().collect()))
        .collect();
    let constrained = axes
        .iter()
        .filter(|(id, _)| involved.contains(id))
        .cloned()
        .collect();
    let safety: Axes<'_> = axes
        .iter()
        .filter(|(id, _)| factors.factors()[*id].safety)
        .cloned()
        .collect();
    let pairs = count::<2>(&axes, &constrained, &rules, &rows);
    let safety_triples = count::<3>(&safety, &constrained, &rules, &rows);
    // Блок Е ТЗ требует тройки только между факторами безопасности.
    let safety_count = safety.len();
    let safety_triples_not_required = (safety_count < 3)
        .then(|| format!("Тройки не требуются: факторов безопасности {safety_count}"));
    let complete = pairs.required == pairs.executed
        && safety_triples.required == safety_triples.executed
        && open_rows.is_empty();
    Ok(Summary {
        scope: if repeat { "repeat" } else { "plan" },
        status: if complete { "completed" } else { "incomplete" },
        pairs,
        safety_triples,
        safety_triples_not_required,
        open_rows,
        worst: worst.map(|(_, worst)| worst),
        refinement,
        acceptance: None,
    })
}

impl Summary {
    pub(super) fn complete(&self) -> bool {
        self.status == "completed"
            && self.acceptance.as_ref().is_none_or(|acceptance| matches!(acceptance.repeat, RepeatStatus::Verified))
    }

    pub(super) fn write(&self, out: &mut impl std::io::Write) -> std::io::Result<()> {
        writeln!(
            out,
            "  Пары · требуется {}, спланировано {}, выполнено {}",
            self.pairs.required, self.pairs.planned, self.pairs.executed
        )?;
        if let Some(reason) = &self.safety_triples_not_required {
            writeln!(out, "  {reason}")?;
        } else {
            let count = &self.safety_triples;
            writeln!(
                out,
                "  Тройки безопасности · требуется {}, спланировано {}, выполнено {}",
                count.required, count.planned, count.executed
            )?;
        }
        for row in &self.open_rows {
            writeln!(out, "  Открытая строка {} · {}", row.run_key, row.reason)?;
        }
        if let Some(worst) = &self.worst {
            writeln!(out, "  Худший опыт · {}", worst.run_key)?;
        }
        if let Some(acceptance) = &self.acceptance {
            writeln!(out, "  Запусков подтверждено {}, зарезервировано {}. Время {} нс",
                acceptance.confirmed_launches, acceptance.reserved_runs, acceptance.elapsed_ns)?;
            match &acceptance.repeat {
                RepeatStatus::Verified => writeln!(out, "  Повтор худшего случая подтверждён через SIMVIEW")?,
                RepeatStatus::NotAnalyzed { reason } => writeln!(out, "  Повтор остаётся открытым · {reason}")?,
            }
        }
        Ok(())
    }
}

#[derive(Serialize)]
#[serde(tag = "status", rename_all = "snake_case")]
pub(super) enum RepeatStatus {
    Verified,
    NotAnalyzed { reason: String },
}

#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct Recorded {
    binding: CampaignBinding,
    outcome: Outcome,
    report: Fingerprint,
    #[serde(default)]
    refinement: Option<RefinedSelection>,
}

#[derive(Deserialize, Serialize)]
pub(super) struct RefinedSelection {
    pub selection: Selection,
    pub order: usize,
}

impl Recorded {
    fn capture(binding: CampaignBinding, row: &RowReport) -> Result<Self, String> {
        Ok(Self {
            binding,
            outcome: row.outcome,
            report: Fingerprint::of(&encode((&row.findings, &row.detail))?),
            refinement: (row.detail["coverage"]["phase"] == "refinement")
                .then(|| serde_json::from_value(row.detail["coverage"].clone()))
                .transpose().map_err(|e| e.to_string())?,
        })
    }

    fn name(key: &str) -> String {
        format!(
            "depth-report-v1-{:016x}",
            Fingerprint::of(key.as_bytes()).as_u64()
        )
    }
}

pub(super) fn recorded_refinement(checkpoint: &Checkpoint, key: &str) -> Result<Option<RefinedSelection>, String> {
    let saved = checkpoint.load_row::<Recorded>(&Recorded::name(key))?;
    if saved.as_ref().is_some_and(|entry| entry.binding.key != key) {
        return Err("чужой ключ в квитанции уточнения".into());
    }
    Ok(saved.and_then(|entry| entry.refinement))
}

pub(super) fn record(
    checkpoint: &Checkpoint,
    binding: CampaignBinding,
    row: &RowReport,
) -> Result<(), String> {
    let name = Recorded::name(&binding.key);
    if let Some(saved) = checkpoint.load_row::<Recorded>(&name)? {
        // Итог сборки после продолжения может измениться. Квитанция первого
        // опубликованного опыта сохраняет исходный контекст для SIMVIEW.
        verify(&saved, &binding, row)?;
        return Ok(());
    }
    checkpoint.save_row(&name, &Recorded::capture(binding, row)?)
}

fn verify(previous: &Recorded, current: &CampaignBinding, row: &RowReport) -> Result<(), String> {
    if current.key != previous.binding.key
        || current.input != previous.binding.input
        || current.playback != previous.binding.playback
        || row.outcome != previous.outcome
        || Fingerprint::of(&encode((&row.findings, &row.detail))?) != previous.report
    {
        return Err("повтор покрытия расходится с исходными входами, журналом или оценкой".into());
    }
    Ok(())
}

pub(super) fn repeat(
    checkpoint: Option<&Checkpoint>,
    binding: CampaignBinding,
    row: &RowReport,
) -> Result<(CampaignBinding, RepeatStatus), String> {
    let saved = checkpoint
        .map(|state| state.load_row::<Recorded>(&Recorded::name(&binding.key)))
        .transpose()?
        .flatten();
    let Some(saved) = saved else {
        return Ok((
            binding,
            RepeatStatus::NotAnalyzed {
                reason: "в контрольной точке отсутствует квитанция исходного опыта".into(),
            },
        ));
    };
    verify(&saved, &binding, row)?;
    Ok((saved.binding, RepeatStatus::Verified))
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Registry {
    version: u32,
    baseline: PathBuf,
    placements: BTreeMap<String, Placement>,
    rows: Vec<HistoricalRow>,
}

#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct Placement {
    world: PathBuf,
    task: PathBuf,
    mode: String,
}

#[derive(Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
enum Classification {
    ConflictingFault,
    ObservationUnavailable,
    InstallationRejected,
}

#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct HistoricalRow {
    key: RunKey,
    classification: Classification,
    outcome: Outcome,
    source: PathBuf,
    reason: String,
    replacement: Option<String>,
    effect: Option<String>,
}

#[derive(Serialize)]
pub(super) struct HistoricalRows {
    baseline: PathBuf,
    registry: PathBuf,
    placements: BTreeMap<String, Placement>,
    rows: Vec<HistoricalRow>,
}

impl HistoricalRows {
    pub(super) fn load(
        base: &Path,
        registry: PathBuf,
        files: &mut processes::Inputs,
    ) -> Result<Self, String> {
        let path = base.join(&registry);
        let parent = processes::dir(&path);
        let mut data: Registry = files.toml(&path)?;
        if data.version != 1 || data.rows.is_empty() {
            return Err("нужен непустой реестр открытых строк версии 1".into());
        }
        files.capture(&parent.join(&data.baseline))?;
        for placement in data.placements.values() {
            if placement.mode.trim().is_empty() {
                return Err("в отдельной постановке открытой строки отсутствует режим".into());
            }
            files.capture(&parent.join(&placement.world))?;
            files.capture(&parent.join(&placement.task))?;
        }
        let mut seen = BTreeSet::new();
        for row in &data.rows {
            if !seen.insert(&row.key)
                || row.outcome != Outcome::NotAnalyzed
                || row.reason.trim().is_empty()
                || row.key.station.split_once('/').is_none()
                || row
                    .replacement
                    .as_ref()
                    .is_some_and(|id| !data.placements.contains_key(id))
            {
                return Err("дубликат или неверное описание исходной открытой строки".into());
            }
            files.capture(&parent.join(&row.source))?;
        }
        data.rows.sort_by(|a, b| a.key.cmp(&b.key));
        Ok(Self {
            baseline: data.baseline,
            registry,
            placements: data.placements,
            rows: data.rows,
        })
    }

    pub(super) fn contains_assembly(&self, id: &str) -> bool {
        self.rows.iter().any(|row| {
            row.key
                .station
                .split_once('/')
                .is_some_and(|(assembly, _)| assembly == id)
        })
    }

    pub(super) fn write(&self, out: &mut impl std::io::Write) -> std::io::Result<()> {
        for row in &self.rows {
            writeln!(
                out,
                "Открытая строка исходной матрицы {:?} · {}",
                row.key, row.reason
            )?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn реестр_сохраняет_исходные_ключи_источники_и_причины() {
        let base = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        let mut files = processes::Inputs::default();
        let report =
            HistoricalRows::load(&base, "devices/stage6/depth-open.toml".into(), &mut files)
                .unwrap();
        // Разбивка исходных 22 строк зафиксирована в ARCHITECTURE.md §7.7.
        assert_eq!(
            report
                .rows
                .iter()
                .filter(|row| matches!(row.classification, Classification::ConflictingFault))
                .count(),
            9
        );
        assert_eq!(
            report
                .rows
                .iter()
                .filter(|row| matches!(row.classification, Classification::ObservationUnavailable))
                .count(),
            12
        );
        assert_eq!(
            report
                .rows
                .iter()
                .filter(|row| matches!(row.classification, Classification::InstallationRejected))
                .count(),
            1
        );
        assert!(report.rows.windows(2).all(|pair| pair[0].key < pair[1].key));
        assert!(
            report
                .rows
                .iter()
                .all(|row| row.outcome == Outcome::NotAnalyzed)
        );
        files.verify().unwrap();
    }
}
