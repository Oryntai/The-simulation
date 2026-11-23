//! Физический уровень: чувствительность приёмника и мощность передатчика.
//!
//! Числа — ESP32-C3 Series Datasheet v2.5 (Espressif, ревизия 11.11.2026), разделы 6.1–6.2:
//! Wi-Fi TX — табл. 6-2, Wi-Fi RX — табл. 6-4, BLE TX — табл. 6-8…6-11 (−24…+20 дБм, шаг 3 дБ),
//! BLE RX — табл. 6-12…6-15 (при PER 30.8 %).
//! Требования стандартов (для сверки наклона «скорость ↔ чувствительность»):
//! IEEE 802.11-2016 табл. 17-18 (OFDM 20 МГц: 6 Мбит/с −82 дБм … 54 Мбит/с −65 дБм) и
//! табл. 19-23 (HT20: MCS0 −82 … MCS7 −64 дБм).

/// Радиорежим.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Phy {
    Ble1M,
    Ble2M,
    /// LE Coded S=2 (500 кбит/с).
    BleCodedS2,
    /// LE Coded S=8 (125 кбит/с) — дальнобойный.
    BleCodedS8,
    WifiG6,
    WifiG54,
    WifiN20Mcs0,
    WifiN20Mcs7,
}

impl Phy {
    pub const ALL: [Phy; 8] = [
        Phy::Ble1M,
        Phy::Ble2M,
        Phy::BleCodedS2,
        Phy::BleCodedS8,
        Phy::WifiG6,
        Phy::WifiG54,
        Phy::WifiN20Mcs0,
        Phy::WifiN20Mcs7,
    ];

    pub fn name(&self) -> &'static str {
        match self {
            Phy::Ble1M => "Bluetooth LE 1M",
            Phy::Ble2M => "Bluetooth LE 2M",
            Phy::BleCodedS2 => "Bluetooth LE Coded 500k",
            Phy::BleCodedS8 => "Bluetooth LE Coded 125k",
            Phy::WifiG6 => "Wi-Fi 802.11g 6 Мбит/с",
            Phy::WifiG54 => "Wi-Fi 802.11g 54 Мбит/с",
            Phy::WifiN20Mcs0 => "Wi-Fi 802.11n MCS0",
            Phy::WifiN20Mcs7 => "Wi-Fi 802.11n MCS7",
        }
    }

    pub fn is_ble(&self) -> bool {
        matches!(
            self,
            Phy::Ble1M | Phy::Ble2M | Phy::BleCodedS2 | Phy::BleCodedS8
        )
    }

    /// Полезная скорость на физическом уровне, бит/с.
    pub fn bitrate_bps(&self) -> f64 {
        match self {
            Phy::Ble1M => 1e6,
            Phy::Ble2M => 2e6,
            Phy::BleCodedS2 => 500e3,
            Phy::BleCodedS8 => 125e3,
            Phy::WifiG6 => 6e6,
            Phy::WifiG54 => 54e6,
            Phy::WifiN20Mcs0 => 6.5e6,
            Phy::WifiN20Mcs7 => 65e6,
        }
    }

    /// Чувствительность ESP32-C3 (типовая), дБм.
    pub fn sensitivity_dbm(&self) -> f64 {
        match self {
            Phy::Ble1M => -97.0,
            Phy::Ble2M => -93.0,
            Phy::BleCodedS2 => -100.0,
            Phy::BleCodedS8 => -105.0,
            Phy::WifiG6 => -93.8,
            Phy::WifiG54 => -76.6,
            Phy::WifiN20Mcs0 => -93.6,
            Phy::WifiN20Mcs7 => -74.8,
        }
    }

    /// Наибольшая мощность передатчика ESP32-C3, дБм.
    pub fn max_tx_dbm(&self) -> f64 {
        match self {
            Phy::Ble1M | Phy::Ble2M | Phy::BleCodedS2 | Phy::BleCodedS8 => 20.0,
            Phy::WifiG6 => 21.0,
            Phy::WifiG54 => 19.0,
            Phy::WifiN20Mcs0 => 20.0,
            Phy::WifiN20Mcs7 => 18.5,
        }
    }

    /// Доля потерянных пакетов, при которой меряют чувствительность.
    pub fn per_at_sensitivity(&self) -> f64 {
        if self.is_ble() {
            0.308 // тестовый пакет 37 байт, BER 0.1 %
        } else {
            0.10 // IEEE 802.11: 10 % для PSDU 1000 (OFDM) байт
        }
    }

    /// Длина тестового пакета для чувствительности, бит.
    pub fn test_packet_bits(&self) -> f64 {
        if self.is_ble() {
            37.0 * 8.0
        } else {
            1000.0 * 8.0
        }
    }

    /// Минимум, который требует стандарт, дБм (None — для BLE сравниваем не со стандартом,
    /// а с энергией на бит).
    pub fn standard_min_sensitivity_dbm(&self) -> Option<f64> {
        match self {
            Phy::WifiG6 => Some(-82.0),
            Phy::WifiG54 => Some(-65.0),
            Phy::WifiN20Mcs0 => Some(-82.0),
            Phy::WifiN20Mcs7 => Some(-64.0),
            _ => None,
        }
    }
}

/// Тепловой шум на входе приёмника: −174 дБм/Гц + 10·lg(полоса) + шум-фактор.
pub fn thermal_noise_dbm(bandwidth_hz: f64, noise_figure_db: f64) -> f64 {
    -174.0 + 10.0 * bandwidth_hz.log10() + noise_figure_db
}
