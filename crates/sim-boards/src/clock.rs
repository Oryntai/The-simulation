//! Точный перевод тактов ядра в такты периферии: остаток хранится как рациональное
//! число, поэтому период таймера не «уплывает» за счёт округления.

/// Накопитель дробной части тиков.
#[derive(Clone, Copy, Debug, Default)]
pub struct TickAcc {
    acc: u128,
}

impl TickAcc {
    /// Сколько целых тиков частоты `f` прошло за `cycles` тактов частоты `fcpu`.
    #[inline]
    pub fn ticks(&mut self, cycles: u64, f: u64, fcpu: u64) -> u64 {
        if f == 0 || fcpu == 0 {
            return 0;
        }
        let t = self.acc + cycles as u128 * f as u128;
        let n = t / fcpu as u128;
        self.acc = t % fcpu as u128;
        n as u64
    }

    /// Сколько тактов `fcpu` нужно, чтобы набралось ещё `n` тиков частоты `f`.
    /// `u64::MAX` — не наберётся никогда (частота нулевая).
    #[inline]
    pub fn cycles_for(&self, n: u64, f: u64, fcpu: u64) -> u64 {
        if f == 0 || fcpu == 0 {
            return u64::MAX;
        }
        let need = (n as u128 * fcpu as u128).saturating_sub(self.acc);
        let c = need.div_ceil(f as u128);
        c.min(u64::MAX as u128) as u64
    }

    /// Сброс дробной части (при перезапуске блока).
    pub fn clear(&mut self) {
        self.acc = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn дробные_тики_не_теряются() {
        // 1 МГц из 72 МГц: 72 000 000 тактов по одному — ровно 1 000 000 тиков.
        let mut a = TickAcc::default();
        let mut n = 0;
        for _ in 0..72_000 {
            n += a.ticks(1000, 1_000_000, 72_000_000);
        }
        assert_eq!(n, 1_000_000);
    }

    #[test]
    fn cycles_for_попадает_ровно_в_тик() {
        let mut a = TickAcc::default();
        assert_eq!(a.ticks(5, 3, 7), 2); // 15/7 = 2, остаток 1
        let c = a.cycles_for(1, 3, 7);
        let mut b = a;
        assert_eq!(b.ticks(c, 3, 7), 1);
        let mut d = a;
        assert_eq!(d.ticks(c - 1, 3, 7), 0);
    }
}
