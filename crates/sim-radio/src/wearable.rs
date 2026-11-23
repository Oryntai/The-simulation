//! Линия «устройство на руке ↔ телефон» в обстановке: расстояние, закрывает ли тело,
//! вода над устройством. Собирает бюджет из моделей [`crate::body`], [`crate::pathloss`],
//! [`crate::dielectric`].

use crate::body::{OFF_BODY_2G36, off_body_db};
use crate::dielectric::{Medium, Permittivity, interface_loss_db};
use crate::link::LinkBudget;
use crate::pathloss::{Surroundings, fspl_db};
use crate::phy::Phy;

/// Частота расчёта, Гц (середина диапазона 2.4 ГГц).
pub const F_2G4: f64 = 2.44e9;

#[derive(Clone, Debug, PartialEq)]
pub struct WearableLink {
    pub phy: Phy,
    pub tx_dbm: f64,
    /// Расстояние до телефона, м.
    pub distance_m: f64,
    pub surroundings: Surroundings,
    /// Тело между устройством и телефоном (рука за спиной, телефон в заднем кармане…).
    pub body_blocks: bool,
    /// Среда над устройством и толщина её слоя (устройство под водой).
    pub medium: Medium,
    pub depth_m: f64,
    /// Мокрая кожа/одежда у антенны: расстройка антенны (оценка 0–6 дБ).
    pub wet_detuning_db: f64,
}

impl WearableLink {
    pub fn budget(&self) -> LinkBudget {
        let d = self.distance_m.max(0.2);
        let d_table = OFF_BODY_2G36[OFF_BODY_2G36.len() - 1].0;
        // Измеренные потери тело → внешний приёмник (CM4) включают близость тела и антенны;
        // дальше 4 м продлеваем показателем обстановки.
        let near = off_body_db(d.min(d_table), !self.body_blocks);
        let far = if d > d_table {
            10.0 * self.surroundings.exponent() * (d / d_table).log10()
        } else {
            0.0
        };
        let sigma = if self.body_blocks { 4.6 } else { 3.6 };
        let mut b = LinkBudget::new(self.phy, self.tx_dbm, sigma).loss(
            if self.body_blocks {
                "трасса с телом (тело закрывает, IEEE 802.15.6 CM4)"
            } else {
                "трасса с телом (прямая видимость, IEEE 802.15.6 CM4)"
            },
            near,
        );
        if far > 0.0 {
            b = b.loss("дальше 4 м по обстановке", far);
        }
        if self.depth_m > 0.0 && self.medium != Medium::Air {
            let eps = self.medium.permittivity(F_2G4);
            b = b
                .loss(
                    "граница вода → воздух",
                    interface_loss_db(eps, Permittivity::VACUUM),
                )
                .loss(
                    &format!("слой воды {:.0} см", self.depth_m * 100.0),
                    eps.attenuation_db_per_m(F_2G4) * self.depth_m,
                );
        }
        if self.wet_detuning_db > 0.0 {
            b = b.loss("мокрая кожа у антенны (оценка)", self.wet_detuning_db);
        }
        b
    }

    /// На сколько измеренная трасса у тела хуже свободного пространства на том же расстоянии.
    pub fn body_excess_over_free_space_db(&self) -> f64 {
        let d = self.distance_m.clamp(1.0, 4.0);
        off_body_db(d, !self.body_blocks) - fspl_db(F_2G4, d)
    }
}

/// Предельная глубина воды над устройством, при которой ещё есть запас `margin_db`
/// (решаем линейно: потери в воде растут пропорционально глубине), м.
pub fn max_depth_m(link: &WearableLink) -> f64 {
    let mut dry = link.clone();
    dry.depth_m = 0.0;
    let base_margin = dry.budget().margin_db();
    let eps = link.medium.permittivity(F_2G4);
    let iface = interface_loss_db(eps, Permittivity::VACUUM);
    let per_m = eps.attenuation_db_per_m(F_2G4);
    ((base_margin - iface) / per_m).max(0.0)
}
