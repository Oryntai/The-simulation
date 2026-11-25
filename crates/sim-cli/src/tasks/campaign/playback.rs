//! Связь событий оценщиков с журналами дочерних прогонов.

use sim_task::{Fingerprint, Outcome, campaign::RunKey, verdict::Finding};

use crate::view::{CampaignEvent, CampaignPlayback};

pub(in crate::tasks) struct Grid {
    pub variant: Option<String>,
    pub hz: u64,
    pub journal: u64,
    pub end_ns: u64,
}

pub(super) fn select(
    child: &RunKey,
    seed: u64,
    grids: impl IntoIterator<Item = Grid>,
    findings: &[&Finding],
) -> Option<CampaignPlayback> {
    grids
        .into_iter()
        .map(|grid| {
            // Время имеет смысл только в журнале, из которого его взял оценщик.
            let failure = findings
                .iter()
                .filter(|f| {
                    f.outcome == Outcome::Fail
                        && f.key.run.as_ref() == Some(child)
                        && f.journal == Some(Fingerprint::from_fnv1a(grid.journal))
                })
                .filter_map(|f| {
                    let at_ns = f.event_ns.filter(|&at| at <= grid.end_ns)?;
                    Some(CampaignEvent {
                        at_ns,
                        label: if f.key.object == f.code {
                            f.code.clone()
                        } else {
                            format!("{} · {}", f.key.object, f.code)
                        },
                        violation: true,
                    })
                })
                .min_by(|a, b| (a.at_ns, &a.label).cmp(&(b.at_ns, &b.label)));
            CampaignPlayback {
                child: child.clone(),
                variant: grid.variant,
                physics_hz: grid.hz,
                seed,
                journal: grid.journal,
                end_ns: grid.end_ns,
                event: failure,
            }
        })
        .reduce(|a, b| {
            let rank = |p: &CampaignPlayback| {
                (
                    p.event.as_ref().is_some_and(|e| e.violation),
                    p.event.is_some(),
                    p.physics_hz,
                )
            };
            if rank(&b) > rank(&a) { b } else { a }
        })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tasks::campaign::{criterion, profile_key};

    #[test]
    fn событие_остаётся_на_своей_сетке_и_в_своём_опыте() {
        // Синтетические отметки двух журналов. Более редкая сетка содержит нарушение.
        let child = profile_key("блок", "пуск", "ось", Vec::new());
        let mut bound = criterion(&child, "ход", "превышение предела");
        bound.event_ns = Some(4);
        bound.journal = Some(Fingerprint::from_fnv1a(1));
        let mut wrong_child = criterion(
            &profile_key("блок", "торможение", "ось", Vec::new()),
            "ход",
            "превышение предела",
        );
        wrong_child.event_ns = Some(1);
        wrong_child.journal = Some(Fingerprint::from_fnv1a(2));
        let mut outside = bound.clone();
        outside.event_ns = Some(6);
        outside.journal = wrong_child.journal;
        let grids = || {
            [
                Grid {
                    variant: None,
                    hz: 1000,
                    journal: 1,
                    end_ns: 5,
                },
                Grid {
                    variant: None,
                    hz: 2000,
                    journal: 2,
                    end_ns: 5,
                },
            ]
        };
        let playback = select(&child, 1, grids(), &[&wrong_child, &outside, &bound]).unwrap();
        assert_eq!((playback.journal, playback.physics_hz), (1, 1000));
        assert_eq!(
            playback.event.map(|e| (e.at_ns, e.violation)),
            Some((4, true))
        );
        let playback = select(&child, 1, grids(), &[&wrong_child, &outside]).unwrap();
        assert_eq!((playback.journal, playback.physics_hz), (2, 2000));
        assert!(playback.event.is_none());
    }
}
