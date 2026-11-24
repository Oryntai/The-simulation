use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use sim_human::{
    Sex,
    scene::{BaseVariant, ParticipantSpec, Proportion, ProportionMeasure, RegionId},
};
use sim_signal::Time;
use sim_world::Pose;

use super::{
    Participant,
    json::{self, Value},
    read_text,
};

#[derive(Clone)]
pub(crate) struct Entry {
    pub id: String,
    pub label: String,
    pub world: PathBuf,
    pub task: PathBuf,
    pub case: String,
    pub participant: Option<Participant>,
    pub placements: BTreeMap<String, Pose>,
    pub inspect_body: String,
    pub force: Option<String>,
    pub peak: Option<String>,
}

impl Entry {
    pub fn peak_signal(&self) -> Option<&str> {
        self.peak.as_deref().or(self.force.as_deref())
    }

    #[cfg(test)]
    pub fn input(&self) -> super::Input {
        super::Input::Task {
            world: self.world.clone(),
            task: self.task.clone(),
            case: Some(self.case.clone()),
            catalog: None,
            matrix: false,
        }
    }
}

pub(crate) struct Suite {
    pub entries: Vec<Entry>,
    pub campaign: PathBuf,
}

impl Suite {
    pub fn load(path: &Path) -> Result<Self, String> {
        let root = json::parse(&read_text(path)?)?;
        all_fields(
            &root,
            &["version", "source", "campaign", "participants", "entries"],
        )?;
        if root.get("version").and_then(Value::u64) != Some(1) {
            return Err("версия списка миров не поддерживается".into());
        }
        required(&root, "source")?;
        let base = path.parent().ok_or("отсутствует каталог списка миров")?;
        let campaign = base.join(required(&root, "campaign")?);
        let participants = root
            .get("participants")
            .and_then(Value::object)
            .ok_or("отсутствует таблица участников")?;
        let mut ids = std::collections::BTreeSet::new();
        let entries = root
            .get("entries")
            .and_then(Value::array)
            .ok_or("отсутствует список миров")?
            .iter()
            .map(|v| {
                all_fields(
                    v,
                    &[
                        "id",
                        "label",
                        "world",
                        "task",
                        "case",
                        "participant",
                        "placements",
                        "inspect_body",
                        "force",
                        "peak",
                    ],
                )?;
                let id = required(v, "id")?.to_owned();
                if !ids.insert(id.clone()) {
                    return Err(format!("повтор строки «{id}»"));
                }
                let participant = optional(v, "participant")?
                    .map(|id| {
                        parse_participant(participants.get(id).ok_or("неизвестный участник")?)
                    })
                    .transpose()?;
                let placements = v
                    .get("placements")
                    .map(|v| {
                        v.object()
                            .ok_or("неверная таблица размещений")?
                            .iter()
                            .map(|(id, v)| Ok((id.to_owned(), parse_pose(v)?)))
                            .collect::<Result<BTreeMap<_, _>, String>>()
                    })
                    .transpose()?
                    .unwrap_or_default();
                Ok(Entry {
                    id,
                    label: required(v, "label")?.to_owned(),
                    world: base.join(required(v, "world")?),
                    task: base.join(required(v, "task")?),
                    case: required(v, "case")?.to_owned(),
                    participant,
                    placements,
                    inspect_body: required(v, "inspect_body")?.to_owned(),
                    force: optional(v, "force")?.map(str::to_owned),
                    peak: optional(v, "peak")?.map(str::to_owned),
                })
            })
            .collect::<Result<Vec<_>, String>>()?;
        if entries.is_empty() {
            return Err("пустой список миров".into());
        }
        Ok(Self { entries, campaign })
    }
}

pub(crate) fn seconds(value: &str) -> Result<Time, String> {
    value
        .trim()
        .parse::<f64>()
        .ok()
        .and_then(Time::from_secs)
        .filter(|t| *t > Time::ZERO)
        .ok_or_else(|| "длительность должна быть положительным конечным числом".into())
}

fn all_fields(v: &Value, allowed: &[&str]) -> Result<(), String> {
    let map = v.object().ok_or("ожидался объект")?;
    if let Some(key) = map.keys().find(|key| !allowed.contains(&key.as_str())) {
        return Err(format!("неизвестное поле «{key}»"));
    }
    Ok(())
}

fn optional<'a>(v: &'a Value, key: &str) -> Result<Option<&'a str>, String> {
    v.get(key)
        .map(|p| {
            p.str()
                .filter(|p| !p.trim().is_empty() && !p.chars().any(char::is_control))
                .ok_or_else(|| format!("неверный текст в поле «{key}»"))
        })
        .transpose()
}

fn required<'a>(v: &'a Value, key: &str) -> Result<&'a str, String> {
    optional(v, key)?.ok_or_else(|| format!("отсутствует поле «{key}»"))
}

fn vecn<const N: usize>(v: &Value) -> Result<[f64; N], String> {
    let v = v.array().ok_or("ожидался вектор")?;
    let v: &[Value; N] = v.try_into().map_err(|_| "неверный размер вектора")?;
    let mut out = [0.0; N];
    for (slot, v) in out.iter_mut().zip(v) {
        *slot = v.f64().ok_or("ожидалось конечное число")?;
    }
    Ok(out)
}

fn parse_pose(v: &Value) -> Result<Pose, String> {
    all_fields(v, &["position_m", "quat_wxyz", "rpy_deg"])?;
    let p = Pose::default();
    Ok(Pose {
        position_m: v
            .get("position_m")
            .map(vecn)
            .transpose()?
            .unwrap_or(p.position_m),
        quat_wxyz: v.get("quat_wxyz").map(vecn).transpose()?,
        rpy_deg: v.get("rpy_deg").map(vecn).transpose()?,
    })
}

fn parse_participant(v: &Value) -> Result<Participant, String> {
    all_fields(
        v,
        &[
            "name",
            "source",
            "population",
            "stature_m",
            "mass_kg",
            "pose",
            "local_dimensions",
        ],
    )?;
    required(v, "source")?;
    let number = |key| {
        v.get(key)
            .and_then(Value::f64)
            .filter(|&x| x > 0.0)
            .ok_or_else(|| format!("неверное значение «{key}»"))
    };
    let population = match required(v, "population")? {
        "male" => Sex::Male,
        "female" => Sex::Female,
        _ => return Err("неизвестная антропометрическая выборка".into()),
    };
    let proportions = v
        .get("local_dimensions")
        .map(|v| {
            v.array()
                .ok_or("ожидался список размеров")?
                .iter()
                .map(|p| {
                    all_fields(p, &["region", "metres", "source"])?;
                    Ok(Proportion {
                        region: RegionId(required(p, "region")?.into()),
                        measure: ProportionMeasure::LocalDimensions(vecn(
                            p.get("metres").ok_or("нет размеров участка")?,
                        )?),
                        source: required(p, "source")?.into(),
                    })
                })
                .collect::<Result<Vec<_>, String>>()
        })
        .transpose()?
        .unwrap_or_default();
    Ok(Participant {
        name: required(v, "name")?.into(),
        spec: ParticipantSpec {
            base: BaseVariant::MsHuman700,
            population,
            stature_m: number("stature_m")?,
            mass_kg: number("mass_kg")?,
            proportions,
        },
        pose: v
            .get("pose")
            .map(parse_pose)
            .transpose()?
            .unwrap_or_default(),
    })
}
