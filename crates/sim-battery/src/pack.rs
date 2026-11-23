//! Сборка S×P и профиль нагрузки: простые поля, которые заполняются из формы.
//!
//! Разбаланс учитывается двумя числами. Разброс DCIR внутри параллельной группы даёт ток
//! худшей ячейки: при `p` ячейках самая «лёгкая» с `R·(1 − s)` стоит рядом с остальными
//! `R·(1 + s)`, и её доля тока равна `(1/(1 − s)) / (1/(1 − s) + (p − 1)/(1 + s))`.
//! Разбаланс напряжения между последовательными группами проверяется в конце заряда.

use crate::cells::CellSpec;
use crate::protection::{BmsSpec, FuseSpec};
use crate::thermal::{EnclosureDesign, SkinInterface};
use crate::wire::WireRun;

/// Рост сопротивления к концу службы: оценка по распространённому критерию «DCIR вырос вдвое».
/// Умножает верхнюю границу сопротивления: это больше нагрева и меньше ток КЗ, на котором
/// должна сработать защита.
pub const EOL_RESISTANCE_FACTOR: f64 = 2.0;

const R_GAS: f64 = 8.314_462_618;
const T25_K: f64 = 298.15;

/// Рост сопротивления на холоде по Аррениусу. При температуре выше 25 °C остаётся равным 1:
/// снижение сопротивления при нагреве в запас не берётся.
pub fn cold_resistance_factor(ea_j_mol: f64, temp_c: f64) -> f64 {
    let f = (ea_j_mol / R_GAS * (1.0 / (temp_c + 273.15) - 1.0 / T25_K)).exp();
    if f > 1.0 { f } else { 1.0 }
}

/// Шаг профиля нагрузки.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LoadStep {
    /// Ток разряда всей батареи, А.
    pub current_a: f64,
    /// Длительность, с.
    pub duration_s: f64,
}

/// Режим заряда.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ChargeProfile {
    /// Ток заряда всей батареи, А.
    pub current_a: f64,
    /// Напряжение зарядного устройства, В.
    pub voltage_v: f64,
    /// Наименьшая температура при заряде, °C.
    pub ambient_min_c: f64,
    /// Наибольшая температура при заряде, °C.
    pub ambient_max_c: f64,
}

/// Профиль нагрузки и условия работы.
#[derive(Clone, Debug, PartialEq)]
pub struct LoadProfile {
    /// Шаги профиля.
    pub steps: Vec<LoadStep>,
    /// Сколько раз повторяется профиль.
    pub cycles: u32,
    /// Наименьшая температура среды, °C.
    pub ambient_min_c: f64,
    /// Наибольшая температура среды, °C.
    pub ambient_max_c: f64,
    /// Наименьшее рабочее напряжение нагрузки, В.
    pub load_v_min: Option<f64>,
    /// Наибольшее допустимое напряжение нагрузки, В.
    pub load_v_max: Option<f64>,
    /// Заряд; `None` — режим не задан.
    pub charge: Option<ChargeProfile>,
}

impl LoadProfile {
    /// Наибольший ток профиля, А.
    pub fn peak_a(&self) -> f64 {
        self.steps
            .iter()
            .fold(0.0, |m, s| if s.current_a > m { s.current_a } else { m })
    }

    /// Действующий (среднеквадратичный) ток за цикл, А.
    pub fn rms_a(&self) -> f64 {
        let (e, t) = self.steps.iter().fold((0.0, 0.0), |(e, t), s| {
            (
                e + s.current_a * s.current_a * s.duration_s,
                t + s.duration_s,
            )
        });
        if t > 0.0 { (e / t).sqrt() } else { 0.0 }
    }

    /// Шаги как пары `(ток, с)`.
    pub fn step_pairs(&self) -> Vec<(f64, f64)> {
        self.steps
            .iter()
            .map(|s| (s.current_a, s.duration_s))
            .collect()
    }
}

/// Конструкция батареи.
#[derive(Clone, Debug, PartialEq)]
pub struct PackDesign {
    /// Ячейка.
    pub cell: CellSpec,
    /// Последовательно.
    pub series: u32,
    /// Параллельно.
    pub parallel: u32,
    /// Относительный разброс DCIR ячеек, ± (0.2 = ±20 %).
    pub cell_resistance_spread: f64,
    /// Наибольшая разница напряжений между последовательными группами, В.
    pub group_voltage_spread_v: f64,
    /// Сумма сопротивлений перемычек и сварки на пути тока, Ом.
    pub interconnect_r_ohm: f64,
    /// Сумма сопротивлений разъёмов до нагрузки, Ом.
    pub contact_r_ohm: f64,
    /// Проводка.
    pub wire: WireRun,
    /// Предохранитель.
    pub fuse: Option<FuseSpec>,
    /// BMS.
    pub bms: Option<BmsSpec>,
    /// Корпус; `None` — данных нет.
    pub enclosure: Option<EnclosureDesign>,
    /// Прилегание к коже; `None` — данных нет.
    pub skin: Option<SkinInterface>,
    /// Батарея носится на теле.
    pub worn_on_body: bool,
}

/// Сводка по сборке.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PackSummary {
    /// Всего ячеек.
    pub cells: u32,
    /// Напряжение полного заряда, В.
    pub v_max: Option<f64>,
    /// Номинальное напряжение, В.
    pub v_nominal: Option<f64>,
    /// Напряжение конца разряда, В.
    pub v_min: Option<f64>,
    /// Гарантированная ёмкость, А·ч.
    pub capacity_ah: Option<f64>,
    /// Номинальная энергия, Вт·ч.
    pub energy_wh: Option<f64>,
    /// Масса ячеек, кг.
    pub mass_kg: Option<f64>,
    /// Внутреннее сопротивление, нижняя оценка, Ом.
    pub r_low_ohm: Option<f64>,
    /// Внутреннее сопротивление свежей батареи, верхняя оценка при 25 °C, Ом.
    pub r_high_ohm: Option<f64>,
    /// Наибольший ток КЗ на клеммах без внешних сопротивлений, А.
    pub i_sc_terminal_a: Option<f64>,
    /// Во сколько раз ток худшей ячейки больше среднего.
    pub worst_cell_share: f64,
}

impl PackDesign {
    /// Всего ячеек.
    pub fn cell_count(&self) -> u32 {
        self.series.saturating_mul(self.parallel)
    }

    /// Отношение S/P.
    pub fn s_over_p(&self) -> f64 {
        self.series as f64 / self.parallel as f64
    }

    /// Во сколько раз ток худшей ячейки группы больше среднего.
    pub fn worst_cell_share(&self) -> f64 {
        let p = self.parallel as f64;
        let s = self.cell_resistance_spread;
        let lo = 1.0 / (1.0 - s);
        let hi = 1.0 / (1.0 + s);
        lo / (lo + (p - 1.0) * hi) * p
    }

    /// Ток худшей ячейки при токе батареи, А.
    pub fn worst_cell_current_a(&self, pack_current_a: f64) -> f64 {
        pack_current_a / self.parallel as f64 * self.worst_cell_share()
    }

    /// Верхняя оценка сопротивления ячейки: конец службы и холод, Ом.
    pub fn cell_r_worst_ohm(&self, temp_c: f64) -> Option<f64> {
        let r = self.cell.r_dc_high_ohm.value?;
        let ea = self.cell.ea_r_j_mol.value?;
        Some(r * EOL_RESISTANCE_FACTOR * cold_resistance_factor(ea, temp_c))
    }

    /// Сводка по сборке.
    pub fn summary(&self) -> PackSummary {
        let s = self.series as f64;
        let p = self.parallel as f64;
        let c = &self.cell;
        let v_max = c.v_charge_max.value.map(|v| v * s);
        let r_low = c.r_low_ohm.value.map(|r| r * s / p);
        PackSummary {
            cells: self.cell_count(),
            v_max,
            v_nominal: c.v_nominal.value.map(|v| v * s),
            v_min: c.v_cutoff.value.map(|v| v * s),
            capacity_ah: c.capacity_min_ah.value.map(|q| q * p),
            energy_wh: match (c.capacity_typ_ah.value, c.v_nominal.value) {
                (Some(q), Some(v)) => Some(q * v * s * p),
                _ => None,
            },
            mass_kg: c.mass_kg.value.map(|m| m * s * p),
            r_low_ohm: r_low,
            r_high_ohm: c.r_dc_high_ohm.value.map(|r| r * s / p),
            i_sc_terminal_a: match (v_max, r_low) {
                (Some(v), Some(r)) => Some(v / r),
                _ => None,
            },
            worst_cell_share: self.worst_cell_share(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::wire::{Conductor, Insulation, WireCooling};

    fn pack(s: u32, p: u32, spread: f64) -> PackDesign {
        PackDesign {
            cell: CellSpec::lg_m50(),
            series: s,
            parallel: p,
            cell_resistance_spread: spread,
            group_voltage_spread_v: 0.0,
            interconnect_r_ohm: 0.0,
            contact_r_ohm: 0.0,
            wire: WireRun {
                conductor: Conductor::Awg(14),
                insulation: Insulation::Silicone,
                length_m: 0.5,
                cooling: WireCooling::FreeAir,
                outer_diameter_mm: None,
            },
            fuse: None,
            bms: None,
            enclosure: None,
            skin: None,
            worn_on_body: true,
        }
    }

    #[test]
    fn summary_matches_hand_calculation() {
        let x = pack(4, 2, 0.0).summary();
        assert_eq!(x.cells, 8);
        assert!((x.v_max.expect("есть") - 16.8).abs() < 1e-12);
        assert!((x.v_nominal.expect("есть") - 14.52).abs() < 1e-12);
        assert!((x.capacity_ah.expect("есть") - 9.7).abs() < 1e-12);
        assert!((x.energy_wh.expect("есть") - 8.0 * 4.9 * 3.63).abs() < 1e-9);
        assert!((x.mass_kg.expect("есть") - 0.548).abs() < 1e-12);
        assert!((x.r_low_ohm.expect("есть") - 0.048).abs() < 1e-12);
        assert!((x.i_sc_terminal_a.expect("есть") - 350.0).abs() < 1e-9);
    }

    #[test]
    fn current_sharing_matches_divider() {
        let d = pack(1, 2, 0.2);
        // 0.8R и 1.2R: доля = (1/0.8)/(1/0.8 + 1/1.2) = 0.6, в 1.2 раза больше средней
        assert!((d.worst_cell_share() - 1.2).abs() < 1e-12);
        assert!((d.worst_cell_current_a(10.0) - 6.0).abs() < 1e-12);
        assert!((pack(3, 1, 0.2).worst_cell_share() - 1.0).abs() < 1e-12);
        assert!((pack(3, 5, 0.0).worst_cell_share() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn cold_factor_grows_only_below_25c() {
        assert_eq!(cold_resistance_factor(15e3, 60.0), 1.0);
        assert!((cold_resistance_factor(15e3, 25.0) - 1.0).abs() < 1e-12);
        let f = cold_resistance_factor(15e3, -20.0);
        assert!((2.5..3.5).contains(&f), "{f}");
    }

    #[test]
    fn rms_and_peak() {
        let l = LoadProfile {
            steps: vec![
                LoadStep {
                    current_a: 10.0,
                    duration_s: 1.0,
                },
                LoadStep {
                    current_a: 0.0,
                    duration_s: 3.0,
                },
            ],
            cycles: 1,
            ambient_min_c: 20.0,
            ambient_max_c: 25.0,
            load_v_min: None,
            load_v_max: None,
            charge: None,
        };
        assert_eq!(l.peak_a(), 10.0);
        assert!((l.rms_a() - 5.0).abs() < 1e-12);
    }
}
