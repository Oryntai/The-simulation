//! Синтетический источник для детерминированного исполнения прошивки.

use crate::BoardFault;
use sim_signal::Rng;

/// Параметры синтетического источника и число чтений в модели.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EspVirtualRngUse {
    pub seed: u64,
    pub reads: u64,
}

// TRM §24.6, RNG_DATA_REG. Регистр доступен только на чтение.
pub(crate) const ADDRESS: u32 = 0x6002_60b0;

#[derive(Clone)]
pub(crate) struct EspVirtualRng {
    usage: EspVirtualRngUse,
    stream: Rng,
}

impl EspVirtualRng {
    pub(crate) fn new(seed: u64) -> Self {
        Self {
            usage: EspVirtualRngUse { seed, reads: 0 },
            stream: Rng::stream(seed, "esp32-c3/rng"),
        }
    }

    pub(crate) fn usage(&self) -> EspVirtualRngUse {
        self.usage
    }

    pub(crate) fn read(&mut self) -> Result<u32, BoardFault> {
        self.usage.reads = self
            .usage
            .reads
            .checked_add(1)
            .ok_or_else(|| BoardFault::Execution("переполнение счётчика чтений RNG".into()))?;
        Ok(self.stream.next_u64() as u32)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn виртуальный_rng_сохраняет_позицию_при_снимке() {
        let seed = crate::BoardId::Esp32C3 as u64; // Зерно синтетической пробы.
        let mut rng = EspVirtualRng::new(seed);
        let first = rng.read().unwrap();
        let mut snapshot = rng.clone();
        let expected: Vec<_> = (0..u32::BITS).map(|_| rng.read().unwrap()).collect();
        assert_eq!(
            (0..u32::BITS)
                .map(|_| snapshot.read().unwrap())
                .collect::<Vec<_>>(),
            expected,
        );
        assert_eq!(
            rng.usage(),
            EspVirtualRngUse {
                seed,
                reads: u64::from(u32::BITS) + 1
            }
        );
        let mut fresh = EspVirtualRng::new(seed);
        assert_eq!(fresh.read(), Ok(first));
        assert_eq!(fresh.usage().reads, 1);
    }
}
