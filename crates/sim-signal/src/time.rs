//! Время — целые наносекунды. Момент k-го такта считается заново от нуля, поэтому частоты,
//! не кратные наносекунде, не накапливают ошибку.

const NS_PER_S: u128 = 1_000_000_000;

/// Момент времени от начала прогона, нс.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Time(pub u64);

impl Time {
    pub const ZERO: Self = Self(0);

    /// Из секунд с округлением до наносекунды. Отрицательное и нечисловое значение недопустимо.
    pub fn from_secs(seconds: f64) -> Option<Self> {
        let ns = seconds * NS_PER_S as f64;
        (ns.is_finite() && ns >= 0.0 && ns < u64::MAX as f64).then(|| Self(ns.round() as u64))
    }

    pub fn as_secs(self) -> f64 {
        self.0 as f64 / NS_PER_S as f64
    }
}

/// Частота как точная дробь `num / den` Гц.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Rate {
    num: u64,
    den: u64,
}

impl Rate {
    pub fn hz(hz: u64) -> Option<Self> {
        Self::ratio(hz, 1)
    }

    pub fn ratio(num: u64, den: u64) -> Option<Self> {
        // Такт короче наносекунды шкала времени не различит.
        (num > 0 && den > 0 && u128::from(num) <= NS_PER_S * u128::from(den))
            .then_some(Self { num, den })
    }

    pub fn as_hz(self) -> f64 {
        self.num as f64 / self.den as f64
    }

    /// Момент такта с номером `k`: ⌊k·10⁹·den / num⌋.
    pub fn tick(self, k: u64) -> Time {
        let ns = u128::from(k) * NS_PER_S * u128::from(self.den) / u128::from(self.num);
        Time(u64::try_from(ns).expect("прогон длиннее шкалы времени"))
    }
}

/// Счётчик тактов одной частоты.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Clock {
    rate: Rate,
    next: u64,
}

impl Clock {
    pub fn new(rate: Rate) -> Self {
        Self { rate, next: 0 }
    }

    pub fn rate(&self) -> Rate {
        self.rate
    }

    /// Когда сработает следующий такт.
    pub fn due(&self) -> Time {
        self.rate.tick(self.next)
    }

    /// Номер следующего такта.
    pub fn index(&self) -> u64 {
        self.next
    }

    pub fn advance(&mut self) {
        self.next += 1;
    }
}

/// Порядок внутри одного момента. Датчик видит мир после шага физики, контроллер видит
/// показания этого же момента, команда применяется к следующему шагу.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Phase {
    Physics,
    Sensors,
    Links,
    Controllers,
    Actuators,
    Evaluators,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn non_integer_period_does_not_drift() {
        // 3 кГц: период 333 333.(3) нс. Сложение округлённого периода уходило бы на 1 мс за 3000 с.
        let r = Rate::hz(3000).unwrap();
        assert_eq!(r.tick(3).0, 1_000_000);
        assert_eq!(r.tick(9_000_000).0, 3_000_000_000_000);
        assert_eq!(r.tick(1).0, 333_333);
        assert_eq!(r.tick(2).0, 666_666);
    }

    #[test]
    fn rational_rate_is_exact() {
        let r = Rate::ratio(44_100, 1).unwrap();
        assert_eq!(r.tick(44_100).0, 1_000_000_000);
        let slow = Rate::ratio(1, 30).unwrap();
        assert_eq!(slow.tick(2).0, 60_000_000_000);
    }

    #[test]
    fn invalid_values_are_refused() {
        assert!(Rate::hz(0).is_none());
        assert!(Rate::ratio(1, 0).is_none());
        assert!(Rate::hz(2_000_000_000).is_none());
        for bad in [f64::NAN, f64::INFINITY, -1.0] {
            assert!(Time::from_secs(bad).is_none());
        }
    }

    #[test]
    fn phases_run_in_declared_order() {
        assert!(Phase::Physics < Phase::Sensors);
        assert!(Phase::Sensors < Phase::Links);
        assert!(Phase::Links < Phase::Controllers);
        assert!(Phase::Controllers < Phase::Actuators);
        assert!(Phase::Actuators < Phase::Evaluators);
    }
}
