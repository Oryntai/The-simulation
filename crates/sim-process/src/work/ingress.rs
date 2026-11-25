//! Связность открытых проходов при неподвижной плоской границе воды.

use std::collections::{BTreeMap, BTreeSet};

use crate::{ProcessError, SourcedRange};

use super::math::{computed, invalid, name};

#[derive(Clone, Debug)]
pub enum Passage {
    Open,
    DeclaredSeal { specification: String },
}

#[derive(Clone, Debug)]
pub struct Opening {
    pub between: [String; 2],
    /// Высота наивысшей точки прохода над общим нулём.
    pub crest_m: SourcedRange,
    pub passage: Passage,
}

#[derive(Clone, Debug)]
pub struct Enclosure {
    pub water_level_m: SourcedRange,
    pub exterior: String,
    pub zones: BTreeSet<String>,
    pub protected: String,
    pub openings: BTreeMap<String, Opening>,
}

#[derive(Clone, Debug)]
pub struct WaterPath {
    /// Порядок отверстий от внешней воды до защищаемой зоны.
    pub openings: Vec<String>,
    pub depth_m: [f64; 2],
    pub sources: Vec<String>,
}

#[derive(Clone, Debug)]
pub enum IngressScreen {
    Fail(WaterPath),
    NotAnalyzed {
        reason: String,
        path: Option<WaterPath>,
    },
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_world::Provenance;

    fn height(m: f64) -> SourcedRange {
        SourcedRange {
            nominal: m,
            min: m,
            max: m,
            provenance: Provenance::Datasheet("Высоты численной оснастки в общей системе".into()),
        }
    }

    #[test]
    fn подъём_прохода_и_заявленное_уплотнение_оставляют_зону_открытой_для_испытаний() {
        let opening = |a: &str, b: &str, crest_m| Opening {
            between: [a.into(), b.into()],
            crest_m: height(crest_m),
            passage: Passage::Open,
        };
        let mut enclosure = Enclosure {
            // Отметки стенда: 20 мм воды, ввод 5 мм, внутренний проход 10 мм.
            water_level_m: height(0.02),
            exterior: "вода".into(),
            zones: ["полость".into(), "защита".into()].into(),
            protected: "защита".into(),
            openings: [
                ("ввод".into(), opening("вода", "полость", 0.005)),
                ("проход".into(), opening("защита", "полость", 0.01)),
                ("обход".into(), opening("вода", "защита", 0.03)),
            ]
            .into(),
        };
        let IngressScreen::Fail(path) = enclosure.assess().unwrap() else {
            panic!("ожидался открытый путь")
        };
        assert_eq!(path.openings, ["ввод", "проход"]);
        assert_eq!(path.depth_m, [0.02 - 0.01; 2]);
        enclosure.openings.get_mut("проход").unwrap().crest_m = height(0.02);
        assert!(matches!(
            enclosure.assess().unwrap(),
            IngressScreen::NotAnalyzed {
                path: Some(WaterPath {
                    depth_m: [0.0, 0.0],
                    ..
                }),
                ..
            }
        ));
        enclosure.openings.get_mut("обход").unwrap().passage = Passage::DeclaredSeal {
            specification: "Уплотнение из чертежа оснастки".into(),
        };
        enclosure.openings.get_mut("проход").unwrap().passage = Passage::DeclaredSeal {
            specification: "Уплотнение межсекционного стыка".into(),
        };
        assert!(
            matches!(enclosure.assess().unwrap(), IngressScreen::NotAnalyzed {
            reason, path: None
        } if reason.contains("требуют испытания"))
        );
        enclosure.openings.get_mut("ввод").unwrap().crest_m = height(f64::NAN);
        assert!(enclosure.assess().is_err());
        enclosure.openings.get_mut("ввод").unwrap().crest_m = height(0.005);
        enclosure.openings.get_mut("проход").unwrap().between[0] = "неизвестная зона".into();
        assert!(enclosure.assess().is_err());
    }
}

impl Enclosure {
    /// Положительная нижняя граница глубины на всём открытом пути означает отказ.
    pub fn assess(&self) -> Result<IngressScreen, ProcessError> {
        self.water_level_m.validate()?;
        for zone in self.zones.iter().chain([&self.exterior, &self.protected]) {
            name(zone)?;
        }
        if !self.zones.contains(&self.protected) || self.zones.contains(&self.exterior) {
            return Err(invalid(
                "неоднозначные границы защищаемой зоны и внешней воды",
            ));
        }
        let mut adjacent = BTreeMap::<&str, Vec<(&str, &str, &Opening)>>::new();
        let mut seals = Vec::new();
        for (id, opening) in &self.openings {
            name(id)?;
            opening.crest_m.validate()?;
            let [a, b] = opening.between.each_ref().map(String::as_str);
            if a == b
                || [a, b]
                    .into_iter()
                    .any(|z| z != self.exterior && !self.zones.contains(z))
            {
                return Err(invalid(
                    "отверстие соединяет неизвестные или одинаковые зоны",
                ));
            }
            match &opening.passage {
                Passage::Open if opening.crest_m.can_prove() => {
                    adjacent.entry(a).or_default().push((b, id, opening));
                    adjacent.entry(b).or_default().push((a, id, opening));
                }
                Passage::Open => (),
                Passage::DeclaredSeal { specification } => {
                    name(specification)?;
                    seals.push(id.as_str());
                }
            }
        }
        if !self.water_level_m.can_prove() {
            return Ok(IngressScreen::NotAnalyzed {
                reason: "уровень воды задан оценкой".into(),
                path: None,
            });
        }
        // В очередь попадают пути в порядке высоты их наивысшего прохода.
        // Первая извлечённая вершина имеет минимальную такую высоту. Равенство
        // разрешается именем зоны, а соседние отверстия идут в порядке имён.
        let mut frontier = vec![(self.exterior.as_str(), None::<[f64; 2]>, Vec::<&str>::new())];
        let mut seen = BTreeSet::new();
        while let Some((i, _)) = frontier.iter().enumerate().min_by(|(_, a), (_, b)| {
            match (a.1, b.1) {
                (Some(a), Some(b)) => a[1].total_cmp(&b[1]),
                (None, None) => std::cmp::Ordering::Equal,
                (None, Some(_)) => std::cmp::Ordering::Less,
                (Some(_), None) => std::cmp::Ordering::Greater,
            }
            .then_with(|| a.0.cmp(b.0))
        }) {
            let (zone, crest, path) = frontier.swap_remove(i);
            if !seen.insert(zone) {
                continue;
            }
            if zone == self.protected {
                let [lo, hi] = crest.ok_or_else(|| invalid("путь без отверстий"))?;
                let depth_m = [self.water_level_m.min - hi, self.water_level_m.max - lo];
                computed(&depth_m)?;
                let path = WaterPath {
                    sources: std::iter::once(format!(
                        "Уровень воды: {:?}",
                        self.water_level_m.provenance
                    ))
                    .chain(path.iter().map(|id| {
                        format!(
                            "Отверстие «{id}»: {:?}",
                            self.openings[*id].crest_m.provenance
                        )
                    }))
                    .collect(),
                    openings: path.into_iter().map(str::to_owned).collect(),
                    depth_m,
                };
                return Ok(if depth_m[0] > 0.0 {
                    IngressScreen::Fail(path)
                } else {
                    IngressScreen::NotAnalyzed {
                        reason: "расчёт высот не подтверждает погружение всего открытого пути"
                            .into(),
                        path: Some(path),
                    }
                });
            }
            for &(next, id, edge) in adjacent.get(zone).into_iter().flatten() {
                if !seen.contains(next) {
                    let mut path = path.clone();
                    path.push(id);
                    let limit = [edge.crest_m.min, edge.crest_m.max];
                    let crest =
                        crest.map_or(limit, |[lo, hi]| [lo.max(limit[0]), hi.max(limit[1])]);
                    frontier.push((next, Some(crest), path));
                }
            }
        }
        let reason = if seals.is_empty() {
            "по данным не подтверждён открытый путь в защищаемую зону".into()
        } else {
            format!(
                "по данным не подтверждён открытый путь; заявленные уплотнения требуют испытания: {}",
                seals.join(", ")
            )
        };
        Ok(IngressScreen::NotAnalyzed { reason, path: None })
    }
}
