//! Бюджет радиолинии: мощность передатчика + усиления − потери → уровень на приёмнике,
//! запас до чувствительности, вероятность обрыва и доля потерянных пакетов.
//!
//! Модель ошибок — оценка: некогерентная двоичная FSK, P_b = ½·exp(−γ/2) (γ = Eb/N0),
//! «привязанная» к паспортной чувствительности: при уровне, равном чувствительности, доля
//! потерь тестового пакета равна той, при которой её мерили (BLE 30.8 %, Wi-Fi 10 %).
//! Медленные замирания (тело, люди, повороты) — логнормальные с σ из измерений.

use crate::phy::Phy;

/// Слагаемое бюджета: усиление (+) или потери (−), дБ.
#[derive(Clone, Debug, PartialEq)]
pub struct Term {
    pub name: String,
    pub db: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct LinkBudget {
    pub phy: Phy,
    pub tx_dbm: f64,
    pub terms: Vec<Term>,
    /// σ логнормальных замираний, дБ.
    pub sigma_db: f64,
}

impl LinkBudget {
    pub fn new(phy: Phy, tx_dbm: f64, sigma_db: f64) -> Self {
        Self {
            phy,
            tx_dbm,
            terms: Vec::new(),
            sigma_db,
        }
    }

    /// Добавить потери (положительное число дБ уменьшает сигнал).
    pub fn loss(mut self, name: &str, db: f64) -> Self {
        self.terms.push(Term {
            name: name.to_string(),
            db: -db,
        });
        self
    }

    /// Добавить усиление (антенны), дБ.
    pub fn gain(mut self, name: &str, db: f64) -> Self {
        self.terms.push(Term {
            name: name.to_string(),
            db,
        });
        self
    }

    pub fn rx_dbm(&self) -> f64 {
        self.tx_dbm + self.terms.iter().map(|t| t.db).sum::<f64>()
    }

    /// Запас до чувствительности, дБ (отрицательный — связи нет в среднем).
    pub fn margin_db(&self) -> f64 {
        self.rx_dbm() - self.phy.sensitivity_dbm()
    }

    /// Вероятность, что медленное замирание опустит сигнал ниже чувствительности.
    pub fn outage_probability(&self) -> f64 {
        if self.sigma_db <= 0.0 {
            return if self.margin_db() < 0.0 { 1.0 } else { 0.0 };
        }
        q_function(self.margin_db() / self.sigma_db)
    }

    /// Доля потерянных пакетов длиной `bits` при среднем уровне (без замираний).
    pub fn per(&self, bits: f64) -> f64 {
        packet_error_rate(self.phy, self.margin_db(), bits)
    }

    /// Доля потерь, усреднённая по логнормальным замираниям.
    pub fn mean_per(&self, bits: f64) -> f64 {
        let m = self.margin_db();
        let s = self.sigma_db;
        if s <= 0.0 {
            return self.per(bits);
        }
        // Интеграл по нормальному распределению запаса: трапеции на ±6σ.
        let n = 240;
        let (mut acc, mut wsum) = (0.0, 0.0);
        for i in 0..=n {
            let z = -6.0 + 12.0 * i as f64 / n as f64;
            let w = (-0.5 * z * z).exp() * if i == 0 || i == n { 0.5 } else { 1.0 };
            acc += w * packet_error_rate(self.phy, m + z * s, bits);
            wsum += w;
        }
        acc / wsum
    }
}

/// Доля потерь пакета длиной `bits` при запасе `margin_db` над чувствительностью режима.
pub fn packet_error_rate(phy: Phy, margin_db: f64, bits: f64) -> f64 {
    let l_ref = phy.test_packet_bits();
    let per_ref = phy.per_at_sensitivity();
    let pb_ref = 1.0 - (1.0 - per_ref).powf(1.0 / l_ref);
    let gamma_ref = -2.0 * (2.0 * pb_ref).ln();
    let gamma = gamma_ref * 10f64.powf(margin_db / 10.0);
    let pb = (0.5 * (-gamma / 2.0).exp()).min(0.5);
    1.0 - (1.0 - pb).powf(bits)
}

/// Q(x) = ½·erfc(x/√2).
pub fn q_function(x: f64) -> f64 {
    0.5 * erfc(x / std::f64::consts::SQRT_2)
}

/// Дополнительная функция ошибок (Numerical Recipes, `erfcc`, Чебышёв; отн. ошибка < 1.2e-7).
pub fn erfc(x: f64) -> f64 {
    let z = x.abs();
    let t = 1.0 / (1.0 + 0.5 * z);
    let ans = t
        * (-z * z - 1.265_512_23
            + t * (1.000_023_68
                + t * (0.374_091_96
                    + t * (0.096_784_18
                        + t * (-0.186_288_06
                            + t * (0.278_868_07
                                + t * (-1.135_203_98
                                    + t * (1.488_515_87
                                        + t * (-0.822_152_23 + t * 0.170_872_77)))))))))
            .exp();
    if x >= 0.0 { ans } else { 2.0 - ans }
}
