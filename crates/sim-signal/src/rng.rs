//! Случайность только из зерна. xoshiro256** (Blackman, Vigna, 2018), начальное состояние
//! разворачивается SplitMix64, как рекомендуют авторы.

use crate::fnv1a;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Rng {
    s: [u64; 4],
    /// Второе значение пары Бокса — Мюллера в битах f64.
    spare: Option<u64>,
}

fn splitmix(x: &mut u64) -> u64 {
    *x = x.wrapping_add(0x9e37_79b9_7f4a_7c15);
    let mut z = *x;
    z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    z ^ (z >> 31)
}

impl Rng {
    /// Независимый поток компонента: одно зерно мира, разные имена.
    pub fn stream(world_seed: u64, component: &str) -> Self {
        let mut x = world_seed ^ fnv1a(component.bytes());
        let s = [
            splitmix(&mut x),
            splitmix(&mut x),
            splitmix(&mut x),
            splitmix(&mut x),
        ];
        Self { s, spare: None }
    }

    pub fn next_u64(&mut self) -> u64 {
        let result = self.s[1].wrapping_mul(5).rotate_left(7).wrapping_mul(9);
        let t = self.s[1] << 17;
        self.s[2] ^= self.s[0];
        self.s[3] ^= self.s[1];
        self.s[1] ^= self.s[2];
        self.s[0] ^= self.s[3];
        self.s[2] ^= t;
        self.s[3] = self.s[3].rotate_left(45);
        result
    }

    /// Равномерно на [0, 1).
    pub fn uniform(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 * (1.0 / (1u64 << 53) as f64)
    }

    /// Нормальное распределение N(0, 1), преобразование Бокса — Мюллера.
    pub fn normal(&mut self) -> f64 {
        if let Some(bits) = self.spare.take() {
            return f64::from_bits(bits);
        }
        let u = 1.0 - self.uniform();
        let v = self.uniform();
        let r = (-2.0 * u.ln()).sqrt();
        let a = std::f64::consts::TAU * v;
        self.spare = Some((r * a.sin()).to_bits());
        r * a.cos()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn same_seed_same_stream_and_names_separate_streams() {
        let mut a = Rng::stream(7, "датчик/1");
        let mut b = Rng::stream(7, "датчик/1");
        let mut c = Rng::stream(7, "датчик/2");
        let mut d = Rng::stream(8, "датчик/1");
        let x: Vec<u64> = (0..64).map(|_| a.next_u64()).collect();
        assert_eq!(x, (0..64).map(|_| b.next_u64()).collect::<Vec<_>>());
        assert_ne!(x, (0..64).map(|_| c.next_u64()).collect::<Vec<_>>());
        assert_ne!(x, (0..64).map(|_| d.next_u64()).collect::<Vec<_>>());
    }

    #[test]
    fn xoshiro_matches_reference_recurrence() {
        // Эталонный код авторов для состояния {1, 2, 3, 4}: первые значения 11520, 0, 1509978240.
        let mut r = Rng { s: [1, 2, 3, 4], spare: None };
        assert_eq!(r.next_u64(), 11520);
        assert_eq!(r.next_u64(), 0);
        assert_eq!(r.next_u64(), 1_509_978_240);
    }

    #[test]
    fn normal_statistics() {
        let mut r = Rng::stream(1, "статистика");
        let n = 400_000;
        let x: Vec<f64> = (0..n).map(|_| r.normal()).collect();
        let mean = x.iter().sum::<f64>() / n as f64;
        let var = x.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / n as f64;
        // Стандартная ошибка среднего 1/√n ≈ 0.0016, дисперсии √(2/n) ≈ 0.0022: допуск 4σ.
        assert!(mean.abs() < 0.0064, "{mean}");
        assert!((var - 1.0).abs() < 0.009, "{var}");
    }

    #[test]
    fn clone_continues_identically() {
        let mut a = Rng::stream(3, "ветвь");
        let _ = a.normal();
        let mut b = a.clone();
        for _ in 0..100 {
            assert_eq!(a.normal().to_bits(), b.normal().to_bits());
        }
    }
}
