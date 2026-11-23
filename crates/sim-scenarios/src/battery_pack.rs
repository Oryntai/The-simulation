//! Батарея экзоскелета для совместной симуляции: S последовательно × P параллельно
//! ячеек LG INR21700-M50 (5 Ач). Модель ячейки — `sim_circuit::battery` (эквивалентная
//! схема: OCV(SOC) + R0 + RC-звенья, параметры Chen 2020; сопротивления растут на холоде
//! по Аррениусу, ёмкость падает по паспорту). Здесь — только сборка в пачку и учёт
//! температуры окружающей среды.

use sim_circuit::battery::{CellParams, Pack as CircuitPack};

pub struct Pack {
    pub series: usize,
    pub parallel: usize,
    pack: CircuitPack,
    temp_c: f64,
}

impl Pack {
    pub fn new(series: usize, parallel: usize, temp_c: f64) -> Self {
        Self {
            series,
            parallel,
            pack: CircuitPack::with_soc(CellParams::lg_m50(), series, parallel, 0.9),
            temp_c,
        }
    }

    /// Номинальное напряжение пачки, В.
    pub fn nominal_voltage(&self) -> f64 {
        self.series as f64 * self.pack.cell().params().nominal_v
    }

    pub fn terminal_voltage(&self) -> f64 {
        self.pack.terminal_voltage()
    }

    pub fn soc(&self) -> f64 {
        self.pack.soc()
    }

    /// Доля номинальной ёмкости, доступная при текущей температуре.
    pub fn capacity_fraction(&self) -> f64 {
        let p = self.pack.cell().params();
        p.available_capacity_ah(self.temp_c) / p.capacity_ah
    }

    /// Ток пачки `i` (А, + разряд) в течение `dt`.
    pub fn step(&mut self, i: f64, dt: f64) {
        let _warnings = self.pack.step(i, dt, self.temp_c);
    }

    pub fn set_temperature(&mut self, t_c: f64) {
        self.temp_c = t_c;
    }

    /// Тепло в батарее сейчас, Вт.
    pub fn heat_w(&self) -> f64 {
        self.pack.heat_w()
    }
}
