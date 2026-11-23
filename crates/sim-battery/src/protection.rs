//! Защита: плавкий предохранитель (время-токовая характеристика, I²t, отключающая
//! способность) и BMS (пороги и задержки по КЗ и перегрузке, пределы MOSFET).
//!
//! Время-токовая характеристика задаётся строками даташита «кратность тока — наименьшее и
//! наибольшее время». Между строками — линейно в логарифмических осях. Выше последней строки
//! наибольшее время считается при постоянном I²t этой строки: реальный предохранитель на
//! больших токах открывается быстрее, поэтому оценка идёт в опасную сторону.
//!
//! Предохранитель серии ATO: Littelfuse 257 Series, каталог 2010 и лист 0257030. Отключающая
//! способность 1000 А при 32 В DC; строки: 110 % — не менее 100 ч; 135 % — 0.75…600 с;
//! 200 % — 0.15…5 с; 350 % — 0.08…0.5 с; 600 % — не более 0.1 с (только до 30 А).

/// Строка время-токовой характеристики.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TccPoint {
    /// Кратность номинального тока.
    pub multiple: f64,
    /// Наименьшее время срабатывания, с.
    pub t_min_s: Option<f64>,
    /// Наибольшее время срабатывания, с.
    pub t_max_s: Option<f64>,
}

/// Плавкий предохранитель.
#[derive(Clone, Debug, PartialEq)]
pub struct FuseSpec {
    /// Название.
    pub name: String,
    /// Номинальный ток, А.
    pub rating_a: f64,
    /// Номинальное напряжение постоянного тока, В; `None` — DC-номинала нет.
    pub voltage_dc_v: Option<f64>,
    /// Отключающая способность при этом напряжении, А.
    pub breaking_a: Option<f64>,
    /// I²t плавления, А²·с.
    pub melt_i2t_a2s: Option<f64>,
    /// Сопротивление в холодном состоянии, Ом.
    pub cold_resistance_ohm: Option<f64>,
    /// Время-токовая характеристика (по возрастанию кратности).
    pub tcc: Vec<TccPoint>,
    /// Расстояние от клемм батареи по плюсовому проводу, м; 0 — внутри батареи.
    pub position_m: f64,
    /// Источник данных.
    pub source: String,
}

fn loglog(points: &[(f64, f64)], x: f64) -> Option<f64> {
    let k = points.partition_point(|p| p.0 <= x);
    if k == 0 || k >= points.len() {
        return None;
    }
    let (a, b) = (points[k - 1], points[k]);
    let s = (x.ln() - a.0.ln()) / (b.0.ln() - a.0.ln());
    Some((a.1.ln() + s * (b.1.ln() - a.1.ln())).exp())
}

impl FuseSpec {
    /// Предохранитель ATO. Доступны только номиналы с проверенными I²t и сопротивлением:
    /// 10, 20, 30 и 40 А. Для 40 А строки 600 % в источнике нет.
    pub fn ato(rating_a: u32) -> Option<FuseSpec> {
        let (r_mohm, i2t) = match rating_a {
            10 => (7.70, 115.0),
            20 => (3.38, 520.0),
            30 => (1.97, 1510.0),
            40 => (1.44, 3310.0),
            _ => return None,
        };
        let mut tcc = vec![
            TccPoint {
                multiple: 1.10,
                t_min_s: Some(360_000.0),
                t_max_s: None,
            },
            TccPoint {
                multiple: 1.35,
                t_min_s: Some(0.75),
                t_max_s: Some(600.0),
            },
            TccPoint {
                multiple: 2.0,
                t_min_s: Some(0.15),
                t_max_s: Some(5.0),
            },
            TccPoint {
                multiple: 3.5,
                t_min_s: Some(0.08),
                t_max_s: Some(0.5),
            },
        ];
        if rating_a <= 30 {
            tcc.push(TccPoint {
                multiple: 6.0,
                t_min_s: None,
                t_max_s: Some(0.1),
            });
        }
        Some(FuseSpec {
            name: format!("ATO {rating_a} А"),
            rating_a: rating_a as f64,
            voltage_dc_v: Some(32.0),
            breaking_a: Some(1000.0),
            melt_i2t_a2s: Some(i2t),
            cold_resistance_ohm: Some(r_mohm * 1e-3),
            tcc,
            position_m: 0.0,
            source: "Littelfuse 257 Series (ATO), каталог 2010 и лист 0257030".into(),
        })
    }

    /// Наименьший ток, при котором срабатывание гарантировано (есть наибольшее время), А.
    pub fn guaranteed_open_a(&self) -> Option<f64> {
        self.tcc
            .iter()
            .find(|p| p.t_max_s.is_some())
            .map(|p| p.multiple * self.rating_a)
    }

    /// Наибольшее время отключения при токе, с. `None` — срабатывание не гарантировано.
    pub fn max_clear_time_s(&self, current_a: f64) -> Option<f64> {
        if !current_a.is_finite() || current_a <= 0.0 || self.rating_a <= 0.0 {
            return None;
        }
        let m = current_a / self.rating_a;
        let pts: Vec<(f64, f64)> = self
            .tcc
            .iter()
            .filter_map(|p| p.t_max_s.map(|t| (p.multiple, t)))
            .collect();
        let first = *pts.first()?;
        let last = *pts.last()?;
        if m < first.0 {
            return None;
        }
        if m >= last.0 {
            return Some(last.1 * (last.0 / m).powi(2));
        }
        loglog(&pts, m)
    }

    /// Наименьшее время плавления при токе, с. `None` — при таком токе плавиться не должен.
    pub fn min_melt_time_s(&self, current_a: f64) -> Option<f64> {
        if !current_a.is_finite() || current_a <= 0.0 || self.rating_a <= 0.0 {
            return None;
        }
        let m = current_a / self.rating_a;
        let pts: Vec<(f64, f64)> = self
            .tcc
            .iter()
            .filter_map(|p| p.t_min_s.map(|t| (p.multiple, t)))
            .collect();
        let first = *pts.first()?;
        let last = *pts.last()?;
        if m < first.0 {
            return None;
        }
        if m >= last.0 {
            let by_row = last.1 * (last.0 / m).powi(2);
            return Some(match self.melt_i2t_a2s {
                Some(i2t) => by_row.min(i2t / (current_a * current_a)),
                None => by_row,
            });
        }
        loglog(&pts, m)
    }
}

/// Силовой MOSFET разрядного ключа BMS.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MosfetSpec {
    /// Предельное напряжение сток-исток, В.
    pub vds_max_v: f64,
    /// Импульсный ток стока, А (весь ключ с учётом параллельных транзисторов).
    pub id_pulse_a: f64,
    /// Длительность импульса, для которой задан этот ток, с.
    pub id_pulse_width_s: f64,
    /// Сопротивление открытого ключа, Ом.
    pub rds_on_ohm: f64,
    /// Допустимая энергия лавинного пробоя, Дж; `None` — данных нет.
    pub avalanche_energy_j: Option<f64>,
}

/// Плата защиты BMS.
#[derive(Clone, Debug, PartialEq)]
pub struct BmsSpec {
    /// Название.
    pub name: String,
    /// Порог КЗ с учётом допуска: наименьший, А.
    pub scd_trip_min_a: f64,
    /// Порог КЗ с учётом допуска: наибольший, А.
    pub scd_trip_max_a: f64,
    /// Наибольшая задержка по КЗ, с.
    pub scd_delay_max_s: f64,
    /// Порог перегрузки с учётом допуска: наименьший, А.
    pub ocd_trip_min_a: f64,
    /// Порог перегрузки с учётом допуска: наибольший, А.
    pub ocd_trip_max_a: f64,
    /// Наименьшая задержка по перегрузке, с.
    pub ocd_delay_min_s: f64,
    /// Наибольшая задержка по перегрузке, с.
    pub ocd_delay_max_s: f64,
    /// Порог защиты от перезаряда на группу, В.
    pub cell_ovp_v: Option<f64>,
    /// Порог защиты от переразряда на группу, В.
    pub cell_uvp_v: Option<f64>,
    /// Разрядный ключ.
    pub mosfet: MosfetSpec,
}

/// Настройки BMS на микросхеме TI BQ769x0.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Bq769x0Config {
    /// Шунт, Ом.
    pub r_sense_ohm: f64,
    /// Порог КЗ на шунте, мВ (22…200).
    pub scd_mv: f64,
    /// Задержка КЗ, мкс: 70, 100, 200 или 400.
    pub scd_delay_us: u32,
    /// Порог перегрузки на шунте, мВ (8…100).
    pub ocd_mv: f64,
    /// Задержка перегрузки, мс (8…1280).
    pub ocd_delay_ms: f64,
    /// Порог перезаряда, В.
    pub cell_ovp_v: Option<f64>,
    /// Порог переразряда, В.
    pub cell_uvp_v: Option<f64>,
    /// Ключ.
    pub mosfet: MosfetSpec,
}

impl BmsSpec {
    /// BMS на TI BQ769x0 с допусками из таблицы электрических характеристик (SLUSBK2I):
    /// смещение порога ±2.5 мВ, масштаб ±10 %, задержка OCD ±20 %, задержка SCD по таблице
    /// (70 → 105 мкс, 100 → 150, 200 → 260, 400 → 520 — наибольшие значения).
    pub fn bq769x0(cfg: &Bq769x0Config) -> Result<BmsSpec, String> {
        if !(cfg.r_sense_ohm.is_finite() && cfg.r_sense_ohm > 0.0) {
            return Err("сопротивление шунта должно быть положительным".into());
        }
        if !(22.0..=200.0).contains(&cfg.scd_mv) {
            return Err(format!(
                "порог КЗ {} мВ вне диапазона 22…200 мВ",
                cfg.scd_mv
            ));
        }
        if !(8.0..=100.0).contains(&cfg.ocd_mv) {
            return Err(format!(
                "порог перегрузки {} мВ вне диапазона 8…100 мВ",
                cfg.ocd_mv
            ));
        }
        if !(8.0..=1280.0).contains(&cfg.ocd_delay_ms) {
            return Err(format!(
                "задержка перегрузки {} мс вне диапазона 8…1280 мс",
                cfg.ocd_delay_ms
            ));
        }
        let scd_delay_max_us = match cfg.scd_delay_us {
            70 => 105.0,
            100 => 150.0,
            200 => 260.0,
            400 => 520.0,
            other => return Err(format!("задержки КЗ {other} мкс нет в таблице BQ769x0")),
        };
        let trip =
            |mv: f64, scale: f64, offset: f64| (mv * scale + offset) * 1e-3 / cfg.r_sense_ohm;
        Ok(BmsSpec {
            name: "TI BQ769x0".into(),
            scd_trip_min_a: trip(cfg.scd_mv, 0.9, -2.5),
            scd_trip_max_a: trip(cfg.scd_mv, 1.1, 2.5),
            scd_delay_max_s: scd_delay_max_us * 1e-6,
            ocd_trip_min_a: trip(cfg.ocd_mv, 0.9, -2.5),
            ocd_trip_max_a: trip(cfg.ocd_mv, 1.1, 2.5),
            ocd_delay_min_s: cfg.ocd_delay_ms * 0.8e-3,
            ocd_delay_max_s: cfg.ocd_delay_ms * 1.2e-3,
            cell_ovp_v: cfg.cell_ovp_v,
            cell_uvp_v: cfg.cell_uvp_v,
            mosfet: cfg.mosfet,
        })
    }

    /// Наибольшее время отключения при токе, с. `None` — BMS такой ток не обнаружит.
    pub fn max_clear_time_s(&self, current_a: f64) -> Option<f64> {
        if !current_a.is_finite() {
            return None;
        }
        if current_a >= self.scd_trip_max_a {
            Some(self.scd_delay_max_s)
        } else if current_a >= self.ocd_trip_max_a {
            Some(self.ocd_delay_max_s)
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ato_table_rows_are_reproduced() {
        let f = FuseSpec::ato(20).expect("номинал есть");
        assert_eq!(f.max_clear_time_s(20.0 * 1.34), None);
        assert!((f.max_clear_time_s(20.0 * 1.35).expect("есть") - 600.0).abs() < 1e-9);
        assert!((f.max_clear_time_s(40.0).expect("есть") - 5.0).abs() < 1e-9);
        assert!((f.max_clear_time_s(70.0).expect("есть") - 0.5).abs() < 1e-9);
        assert!((f.max_clear_time_s(120.0).expect("есть") - 0.1).abs() < 1e-9);
        assert_eq!(f.guaranteed_open_a(), Some(27.0));
    }

    #[test]
    fn unverified_rating_is_refused() {
        assert!(FuseSpec::ato(15).is_none());
    }

    #[test]
    fn clearing_time_decreases_with_current() {
        let f = FuseSpec::ato(30).expect("номинал есть");
        let mut last = f64::INFINITY;
        for k in 0..200 {
            let i = 41.0 * 1.04f64.powi(k);
            let t = f.max_clear_time_s(i).expect("выше порога");
            assert!(t <= last + 1e-12, "{i}: {t} > {last}");
            last = t;
        }
    }

    #[test]
    fn non_finite_current_never_gets_a_clearing_time() {
        let f = FuseSpec::ato(10).expect("номинал есть");
        for x in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, 0.0, -5.0] {
            assert_eq!(f.max_clear_time_s(x), None);
            assert_eq!(f.min_melt_time_s(x), None);
        }
    }

    #[test]
    fn bq_trip_uses_tolerance_on_the_dangerous_side() {
        let cfg = Bq769x0Config {
            r_sense_ohm: 0.002,
            scd_mv: 44.0,
            scd_delay_us: 100,
            ocd_mv: 17.0,
            ocd_delay_ms: 160.0,
            cell_ovp_v: Some(4.25),
            cell_uvp_v: Some(2.8),
            mosfet: MosfetSpec {
                vds_max_v: 40.0,
                id_pulse_a: 400.0,
                id_pulse_width_s: 1e-4,
                rds_on_ohm: 0.002,
                avalanche_energy_j: None,
            },
        };
        let b = BmsSpec::bq769x0(&cfg).expect("настройки допустимы");
        assert!((b.scd_trip_max_a - (44.0 * 1.1 + 2.5) * 1e-3 / 0.002).abs() < 1e-9);
        assert!((b.scd_delay_max_s - 150e-6).abs() < 1e-12);
        assert!((b.ocd_delay_max_s - 0.192).abs() < 1e-12);
        let mut bad = cfg;
        bad.scd_delay_us = 123;
        assert!(BmsSpec::bq769x0(&bad).is_err());
    }
}
