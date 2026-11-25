use std::num::NonZeroUsize;
use std::sync::Mutex;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::thread;

/// Один бюджет для сборок, опытов и их строк. Вызвавший поток сам исполняет
/// строки и может занимать свободные места общего бюджета для помощников.
pub(crate) struct Workers {
    limit: usize,
    spare: AtomicUsize,
}

impl Workers {
    pub(super) fn new(limit: NonZeroUsize) -> Self {
        Self {
            limit: limit.get(),
            spare: AtomicUsize::new(limit.get() - 1),
        }
    }

    pub(super) fn cpu_count() -> NonZeroUsize {
        thread::available_parallelism().unwrap_or(NonZeroUsize::MIN)
    }

    pub(super) fn campaign_jobs() -> NonZeroUsize {
        // В тесте взаимодействия один процесс занимал около 2 ГиБ.
        // Четыре места оставляют запас в бюджете кампании около 10 ГиБ.
        const MEMORY_LIMIT: NonZeroUsize = NonZeroUsize::new(4).unwrap();
        Self::cpu_count().min(MEMORY_LIMIT)
    }

    fn reserve(&self) -> Option<Slot<'_>> {
        self.spare
            .try_update(Ordering::Relaxed, Ordering::Relaxed, |spare| {
                spare.checked_sub(1)
            })
            .ok()
            .map(|_| Slot(&self.spare))
    }

    pub(super) fn map<T: Send, R: Send>(
        &self,
        items: Vec<T>,
        local: Option<NonZeroUsize>,
        run: impl Fn(T) -> Result<R, String> + Sync,
    ) -> Result<Vec<R>, String> {
        let limit = self
            .limit
            .min(local.map_or(self.limit, NonZeroUsize::get))
            .min(items.len());
        if limit <= 1 {
            return items.into_iter().map(run).collect();
        }
        let pending = Mutex::new(items.into_iter().enumerate());
        let next = || pending.lock().expect("очередь исполнителей").next();
        let run = &run;
        thread::scope(|scope| {
            let mut helpers = Vec::new();
            let mut results = Vec::new();
            while let Some((index, item)) = next() {
                // Повторяем набор помощников между строками: соседние сборки
                // могут освободить места, пока исполнялась текущая строка.
                while helpers.len() < limit - 1
                    && pending.lock().expect("очередь исполнителей").len() > 0
                {
                    let Some(slot) = self.reserve() else { break };
                    helpers.push(
                        thread::Builder::new()
                            .spawn_scoped(scope, move || {
                                let _slot = slot;
                                std::iter::from_fn(next)
                                    .map(|(index, item)| (index, run(item)))
                                    .collect::<Vec<_>>()
                            })
                            .map_err(|error| format!("не удалось создать исполнителя: {error}"))?,
                    );
                }
                results.push((index, run(item)));
            }
            for helper in helpers {
                results.extend(
                    helper
                        .join()
                        .map_err(|_| "ошибка исполнителя кампании или матрицы")?,
                );
            }
            results.sort_unstable_by_key(|(index, _)| *index);
            results.into_iter().map(|(_, result)| result).collect()
        })
    }
}

struct Slot<'a>(&'a AtomicUsize);

impl Drop for Slot<'_> {
    fn drop(&mut self) {
        self.0.fetch_add(1, Ordering::Relaxed);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Condvar;
    use std::time::Duration;

    #[test]
    fn общий_лимит_охватывает_вложенные_очереди() {
        // Две группы по две строки, два общих места для работы.
        let limit = NonZeroUsize::new(2).unwrap();
        let workers = Workers::new(limit);
        let active = AtomicUsize::new(0);
        let peak = AtomicUsize::new(0);
        let (ready, signal) = (Mutex::new(false), Condvar::new());
        let results = workers
            .map(vec![[0, 1], [2, 3]], None, |group| {
                workers.map(group.to_vec(), None, |value| {
                    let count = active.fetch_add(1, Ordering::SeqCst) + 1;
                    peak.fetch_max(count, Ordering::SeqCst);
                    let mut ready = ready.lock().unwrap();
                    if count >= limit.get() {
                        *ready = true;
                        signal.notify_all();
                    }
                    // Таймаут даёт явный провал теста, если второй поток отсутствует.
                    let (ready, _) = signal
                        .wait_timeout_while(ready, Duration::from_secs(5), |ready| !*ready)
                        .unwrap();
                    active.fetch_sub(1, Ordering::SeqCst);
                    if !*ready {
                        return Err("строки не пересеклись во времени".into());
                    }
                    Ok(value)
                })
            })
            .unwrap();
        assert_eq!(results, [vec![0, 1], vec![2, 3]]);
        assert_eq!(peak.load(Ordering::SeqCst), limit.get());
    }
}
