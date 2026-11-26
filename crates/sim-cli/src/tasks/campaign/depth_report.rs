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
    pairs: Count,
    safety_triples: Count,
    open_rows: Vec<OpenRow>,
    pub(super) worst: Option<Worst>,
}

#[derive(Serialize)]
struct OpenRow {
    run_key: String,
    key: Option<RunKey>,
    order: Option<usize>,
    reason: String,
}

#[derive(Deserialize, Serialize)]
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
    parameters: Option<BTreeMap<String, f64>>,
    playback: Option<CampaignPlayback>,
    ranking: &'static str,
    margin: RepeatStatus,
}

pub(super) fn is_planned(runs: &BTreeMap<String, RowReport>) -> bool {
    runs.values()
        .any(|row| row.detail["coverage"]["order"].is_u64())
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
        required += usize::from(admissible(
            constrained,
            rules,
            &mut tuple.iter().copied().collect(),
        ));
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
            || row.executed != usize::from(complete)
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
        // Исход оценщика имеет приоритет. При равном исходе выбираем первый
        // завершённый опыт с доступным журналом в фактическом порядке плана.
        if complete {
            let rank = (
                row.outcome,
                row.playback.is_some(),
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
                worst = Some((rank, Worst {
                    run_key: run_key.clone(),
                    key: receipt.key.clone(),
                    selection: receipt.selection.clone(),
                    plan_input: receipt.input,
                    input: row.input,
                    outcome: row.outcome,
                    parameters,
                    playback: row.playback.clone(),
                    ranking: "outcome",
                    margin: RepeatStatus::NotAnalyzed {
                        reason: "в квитанции оценщика отсутствует привязанный нормированный запас и окно критерия".into(),
                    },
                }));
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
    let safety = axes
        .iter()
        .filter(|(id, _)| factors.factors()[*id].safety)
        .cloned()
        .collect();
    Ok(Summary {
        scope: if repeat { "repeat" } else { "plan" },
        pairs: count::<2>(&axes, &constrained, &rules, &rows),
        safety_triples: count::<3>(&safety, &constrained, &rules, &rows),
        open_rows,
        worst: worst.map(|(_, worst)| worst),
    })
}

impl Summary {
    pub(super) fn complete(&self) -> bool {
        self.pairs.required == self.pairs.executed
            && self.safety_triples.required == self.safety_triples.executed
            && self.open_rows.is_empty()
    }

    pub(super) fn write(&self, out: &mut impl std::io::Write) -> std::io::Result<()> {
        for (name, count) in [
            ("Пары", &self.pairs),
            ("Тройки безопасности", &self.safety_triples),
        ] {
            writeln!(
                out,
                "  {name} · требуется {}, спланировано {}, выполнено {}",
                count.required, count.planned, count.executed
            )?;
        }
        for row in &self.open_rows {
            writeln!(out, "  Открытая строка {} · {}", row.run_key, row.reason)?;
        }
        if let Some(worst) = &self.worst {
            writeln!(out, "  Худший выполненный опыт · {}", worst.run_key)?;
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
}

impl Recorded {
    fn capture(binding: CampaignBinding, row: &RowReport) -> Result<Self, String> {
        Ok(Self {
            binding,
            outcome: row.outcome,
            report: Fingerprint::of(&encode((&row.findings, &row.detail))?),
        })
    }

    fn name(key: &str) -> String {
        format!(
            "depth-report-v1-{:016x}",
            Fingerprint::of(key.as_bytes()).as_u64()
        )
    }
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
