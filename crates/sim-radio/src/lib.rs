//! `sim-radio` — радио: бюджет линии Wi-Fi/Bluetooth, потери в теле, воде и помещении,
//! пакетный канал для прошивки.
//!
//! * [`dielectric`] — проницаемость воды (две независимые модели) и тканей, затухание;
//! * [`pathloss`] — свободное пространство (Фриис и ITU-R P.525), логарифмическая модель;
//! * [`body`] — каналы на теле по измерениям IEEE 802.15.6 (CM3, CM4, огибание тела);
//! * [`phy`] — чувствительность и мощность ESP32-C3 по даташиту, требования стандартов;
//! * [`link`] — бюджет, запас, обрыв, потери пакетов;
//! * [`channel`] — пакетный канал BLE/Wi-Fi во времени (задержки, повторы, разрыв);
//! * [`wearable`] — сборка бюджета «устройство на руке ↔ телефон».

pub mod body;
pub mod channel;
pub mod dielectric;
pub mod link;
pub mod pathloss;
pub mod phy;
pub mod wearable;

use sim_verify::{Check, Tolerance};

/// Скорость света, м/с.
pub const C: f64 = 299_792_458.0;
/// Электрическая постоянная, Ф/м.
pub const EPS0: f64 = 8.854_187_812_8e-12;

/// Все сверки этого блока с эталоном (вторым независимым способом).
pub fn reference_checks() -> Vec<Check> {
    use dielectric::*;
    use phy::Phy;
    let f = 2.45e9;
    let mut v = Vec::new();

    v.push(Check {
        id: "radio_fspl",
        title: "Свободное пространство, 2.45 ГГц, 10 м".into(),
        simulated: pathloss::fspl_db(f, 10.0),
        reference: pathloss::fspl_p525_db(f * 1e-6, 0.01),
        unit: "дБ",
        method: "Фриис: 20·lg(4π·d/λ)".into(),
        reference_method: "ITU-R P.525-4: 32.4 + 20·lg f[МГц] + 20·lg d[км]".into(),
        tolerance: Tolerance::Absolute(0.06),
        note: "Константа 32.4 в ITU округлена (точно 32.45) — отсюда ~0.05 дБ.".into(),
    });

    let p840 = water_itu_p840(20.0, f);
    let ks = water_klein_swift(20.0, 0.0, 0.0, f);
    v.push(Check {
        id: "radio_water_eps_re",
        title: "Пресная вода 20 °C, 2.45 ГГц: ε'".into(),
        simulated: ks.re,
        reference: p840.re,
        unit: "",
        method: "Klein & Swift 1977 (одиночный Дебай)".into(),
        reference_method: "ITU-R P.840 (двойной Дебай, Liebe 1991)".into(),
        tolerance: Tolerance::Relative(0.02),
        note: "Две независимые подгонки к разным лабораторным данным.".into(),
    });
    v.push(Check {
        id: "radio_water_eps_im",
        title: "Пресная вода 20 °C, 2.45 ГГц: ε'' (потери)".into(),
        simulated: ks.im,
        reference: p840.im,
        unit: "",
        method: "Klein & Swift 1977".into(),
        reference_method: "ITU-R P.840".into(),
        tolerance: Tolerance::Relative(0.05),
        note: "".into(),
    });
    v.push(Check {
        id: "radio_water_atten",
        title: "Затухание в пресной воде 20 °C, 2.45 ГГц".into(),
        simulated: ks.attenuation_db_per_m(f),
        reference: p840.attenuation_db_per_m(f),
        unit: "дБ/м",
        method: "α по Klein & Swift".into(),
        reference_method: "α по ITU-R P.840".into(),
        tolerance: Tolerance::Relative(0.05),
        note: "≈ 2.5 дБ на сантиметр: под водой BLE/Wi-Fi пропадают за считанные сантиметры."
            .into(),
    });
    v.push(Check {
        id: "radio_sea_sigma",
        title: "Проводимость морской воды 35 ‰ при 15 °C".into(),
        simulated: seawater_conductivity_klein_swift(15.0, 35.0),
        reference: 4.2914,
        unit: "См/м",
        method: "Klein & Swift 1977".into(),
        reference_method: "Определение практической солёности PSS-78: C(35, 15 °C) = 42.914 мСм/см"
            .into(),
        tolerance: Tolerance::Relative(0.01),
        note: "".into(),
    });
    let (e, s, depth) = JOHNSON_GUY_MUSCLE_2450;
    v.push(Check {
        id: "radio_muscle_depth",
        title: "Глубина проникновения 2.45 ГГц в мышцу".into(),
        simulated: Permittivity::from_conductivity(e, s, 2.45e9).penetration_depth_m(2.45e9),
        reference: depth,
        unit: "м",
        method: "Наша формула затухания с ε', σ из той же статьи".into(),
        reference_method: "Johnson & Guy 1972, таблица (мышца, 2450 МГц): 1.70 см".into(),
        tolerance: Tolerance::Relative(0.04),
        note: "Проверяет формулу α; сами ткани берём по IT'IS (мышца 52.7 / 1.74 См/м).".into(),
    });

    // Наклон «скорость ↔ чувствительность»: паспорт ESP32-C3 против стандарта/теории.
    let d = |a: Phy, b: Phy| b.sensitivity_dbm() - a.sensitivity_dbm();
    let spec = |a: Phy, b: Phy| {
        b.standard_min_sensitivity_dbm().unwrap() - a.standard_min_sensitivity_dbm().unwrap()
    };
    for (id, a, b) in [
        ("radio_wifi_g_slope", Phy::WifiG6, Phy::WifiG54),
        ("radio_wifi_n_slope", Phy::WifiN20Mcs0, Phy::WifiN20Mcs7),
    ] {
        v.push(Check {
            id,
            title: format!("Цена скорости: {} → {}", a.name(), b.name()),
            simulated: d(a, b),
            reference: spec(a, b),
            unit: "дБ",
            method: "Разница чувствительностей в даташите ESP32-C3 v2.5".into(),
            reference_method: "Разница минимумов IEEE 802.11-2016 (табл. 17-18 / 19-23)".into(),
            tolerance: Tolerance::Absolute(1.5),
            note: "Быстрая модуляция требует большего сигнала — абсолютные числа чипа лучше \
                   стандарта, наклон тот же."
                .into(),
        });
    }
    for (id, b, gain) in [
        ("radio_ble_2m", Phy::Ble2M, -10.0 * 2f64.log10()),
        ("radio_ble_s2", Phy::BleCodedS2, 10.0 * 2f64.log10()),
        ("radio_ble_s8", Phy::BleCodedS8, 10.0 * 8f64.log10()),
    ] {
        v.push(Check {
            id,
            title: format!("Выигрыш {} над BLE 1M", b.name()),
            simulated: -d(Phy::Ble1M, b),
            reference: gain,
            unit: "дБ",
            method: "Разница чувствительностей в даташите ESP32-C3".into(),
            reference_method: "Энергия на бит: 10·lg(отношение скоростей)".into(),
            tolerance: Tolerance::Absolute(1.5),
            note: "Кодированный режим медленнее — каждому биту достаётся больше энергии.".into(),
        });
    }

    // Задержка BLE: Монте-Карло канала против аналитики.
    let (interval, per) = (0.03, 0.3);
    let mut ch = channel::PacketChannel::new(
        channel::Protocol::Ble {
            interval_s: interval,
            supervision_timeout_s: 1e9,
            adv_interval_s: 0.1,
        },
        7,
    );
    ch.set_link(per, -80.0);
    let mut rng = channel::Rng::new(11);
    let mut t = 0.0;
    for _ in 0..20_000 {
        t += 0.2 + 0.1 * rng.uniform();
        ch.send(t, channel::Dir::DeviceToPhone, &[1]);
        ch.advance(t + 0.19);
    }
    ch.advance(t + 10.0);
    v.push(Check {
        id: "radio_ble_latency",
        title: "Средняя задержка BLE: интервал 30 мс, 30 % потерь попытки".into(),
        simulated: ch.stats.mean_latency_s(),
        reference: channel::ble_mean_latency_analytic(interval, per),
        unit: "с",
        method: "Моделирование канала, 20 000 пакетов".into(),
        reference_method: "T/2 + T·p/(1 − p)".into(),
        tolerance: Tolerance::Relative(0.03),
        note: "Ожидание события + геометрическое число повторов.".into(),
    });
    v
}

pub mod timed;
