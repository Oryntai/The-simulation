//! Потери на трассе в воздухе.
//!
//! * Свободное пространство — формула Фрииса: L = (4π·d/λ)².
//! * Та же величина в виде ITU-R P.525-4: L = 32.4 + 20·lg f[МГц] + 20·lg d[км] (второй способ).
//! * Логарифмическая модель L(d) = L(1 м) + 10·n·lg(d/1 м) с показателем n по типу помещения
//!   (Rappaport, «Wireless Communications», 2-е изд., табл. 4.2: свободное пространство 2,
//!   в здании прямая видимость 1.6–1.8, в здании с препятствиями 4–6, на заводе с препятствиями 2–3).

use crate::C;
use std::f64::consts::PI;

pub fn wavelength_m(f_hz: f64) -> f64 {
    C / f_hz
}

/// Потери свободного пространства по Фриису (изотропные антенны), дБ.
pub fn fspl_db(f_hz: f64, d_m: f64) -> f64 {
    20.0 * (4.0 * PI * d_m / wavelength_m(f_hz)).log10()
}

/// То же по ITU-R P.525-4, формула (6): f в МГц, d в км.
pub fn fspl_p525_db(f_mhz: f64, d_km: f64) -> f64 {
    32.4 + 20.0 * f_mhz.log10() + 20.0 * d_km.log10()
}

/// Тип обстановки для логарифмической модели.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Surroundings {
    /// Открытое место, прямая видимость: n = 2.
    OpenLos,
    /// Помещение, прямая видимость: n = 1.7 (коридорный эффект).
    IndoorLos,
    /// Помещение со стенами и людьми между: n = 3.5 (нижний край «здание с препятствиями»
    /// у Rappaport 4–6 взят мягче — оценка для квартиры/лаборатории).
    IndoorObstructed,
}

impl Surroundings {
    pub fn exponent(&self) -> f64 {
        match self {
            Surroundings::OpenLos => 2.0,
            Surroundings::IndoorLos => 1.7,
            Surroundings::IndoorObstructed => 3.5,
        }
    }

    pub fn name(&self) -> &'static str {
        match self {
            Surroundings::OpenLos => "открытое место",
            Surroundings::IndoorLos => "помещение, видно",
            Surroundings::IndoorObstructed => "помещение, стены",
        }
    }
}

/// Логарифмическая модель: свободное пространство до 1 м, дальше показатель n.
pub fn log_distance_db(f_hz: f64, d_m: f64, s: Surroundings) -> f64 {
    let d = d_m.max(0.01);
    if d <= 1.0 {
        fspl_db(f_hz, d)
    } else {
        fspl_db(f_hz, 1.0) + 10.0 * s.exponent() * d.log10()
    }
}
