//! Короткое замыкание в выбранной точке цепи.
//!
//! Цепь: ячейки → перемычки → ключ BMS → предохранитель → плюсовой провод → разъём → нагрузка
//! → минусовой провод. Для каждой точки считаются два тока, потому что опасная сторона разная:
//! * **наибольший** — полный заряд, нижняя оценка сопротивления ячеек, холодный провод.
//!   Перемычки, контакты, предохранитель и ключ в него не входят: их номиналы заданы без
//!   нижнего допуска, а завышенное сопротивление дало бы ложный запас. По этому току
//!   проверяются отключающая способность и ключ BMS;
//! * **наименьший** — конец разряда, холод, конец службы, провод при предельной температуре,
//!   контакт с заданным сопротивлением. По нему проверяется, сработает ли защита вообще.
//!
//! Допущение: ячейка — ЭДС за постоянным сопротивлением; падение ЭДС за время КЗ и энергия,
//! ограниченная ёмкостью, в запас не берутся.

use crate::pack::{LoadProfile, PackDesign};

/// Точка короткого замыкания.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FaultPoint {
    /// На клеммах батареи.
    Terminals,
    /// Сразу после предохранителя.
    AfterFuse,
    /// На нагрузке, в конце провода.
    AtLoad,
}

/// Все точки короткого замыкания.
pub const ALL_FAULT_POINTS: [FaultPoint; 3] = [
    FaultPoint::Terminals,
    FaultPoint::AfterFuse,
    FaultPoint::AtLoad,
];

impl FaultPoint {
    /// Название для интерфейса.
    pub fn name(self) -> &'static str {
        match self {
            FaultPoint::Terminals => "на клеммах",
            FaultPoint::AfterFuse => "после предохранителя",
            FaultPoint::AtLoad => "на нагрузке",
        }
    }
}

/// Токи короткого замыкания в точке.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FaultCurrents {
    /// Точка.
    pub point: FaultPoint,
    /// Наибольший ток, А.
    pub i_max_a: f64,
    /// Наименьший ток, А.
    pub i_min_a: f64,
    /// Сопротивление контура для наибольшего тока, Ом.
    pub r_loop_min_ohm: f64,
    /// Сопротивление контура для наименьшего тока, Ом.
    pub r_loop_max_ohm: f64,
    /// Предохранитель стоит на пути этого тока.
    pub fuse_in_path: bool,
    /// Длина провода в контуре, м.
    pub wire_loop_m: f64,
}

/// Токи КЗ в точке. `None` — нет напряжения или сопротивления ячейки.
pub fn short_circuit(
    pack: &PackDesign,
    load: &LoadProfile,
    point: FaultPoint,
) -> Option<FaultCurrents> {
    let s = pack.series as f64;
    let v_max = pack.cell.v_charge_max.value? * s;
    let v_min = pack.cell.v_cutoff.value? * s;
    let r_cells_low = pack.cell.r_low_ohm.value? * pack.s_over_p();
    let r_cells_high = pack.cell_r_worst_ohm(load.ambient_min_c)? * pack.s_over_p();

    let fuse_at_terminals = pack.fuse.as_ref().is_some_and(|f| f.position_m <= 0.0);
    let (fuse_in_path, wire_loop_m, contact) = match point {
        FaultPoint::Terminals => (fuse_at_terminals, 0.0, 0.0),
        FaultPoint::AfterFuse => (
            pack.fuse.is_some(),
            2.0 * pack.fuse.as_ref().map_or(0.0, |f| f.position_m),
            0.0,
        ),
        FaultPoint::AtLoad => (
            pack.fuse.is_some(),
            2.0 * pack.wire.length_m,
            pack.contact_r_ohm,
        ),
    };
    let r_fuse = if fuse_in_path {
        pack.fuse
            .as_ref()
            .and_then(|f| f.cold_resistance_ohm)
            .unwrap_or(0.0)
    } else {
        0.0
    };
    let r_fet = pack.bms.as_ref().map_or(0.0, |b| b.mosfet.rds_on_ohm);
    let common = pack.interconnect_r_ohm + r_fuse + r_fet;

    let t_wire_cold = if load.ambient_min_c < 20.0 {
        load.ambient_min_c
    } else {
        20.0
    };
    let r_wire_cold = pack.wire.loop_resistance_ohm(wire_loop_m, t_wire_cold);
    let r_wire_hot = pack
        .wire
        .loop_resistance_ohm(wire_loop_m, pack.wire.insulation.continuous_c());

    let r_loop_min_ohm = r_cells_low + r_wire_cold;
    let r_loop_max_ohm = r_cells_high + common + r_wire_hot + contact;
    Some(FaultCurrents {
        point,
        i_max_a: v_max / r_loop_min_ohm,
        i_min_a: v_min / r_loop_max_ohm,
        r_loop_min_ohm,
        r_loop_max_ohm,
        fuse_in_path,
        wire_loop_m,
    })
}

/// Токи КЗ во всех точках. Пустой вектор — нет данных ячейки.
pub fn fault_table(pack: &PackDesign, load: &LoadProfile) -> Vec<FaultCurrents> {
    ALL_FAULT_POINTS
        .iter()
        .filter_map(|&p| short_circuit(pack, load, p))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cells::CellSpec;
    use crate::pack::EOL_RESISTANCE_FACTOR;
    use crate::protection::FuseSpec;
    use crate::wire::{Conductor, Insulation, WireCooling, WireRun};

    fn pack() -> PackDesign {
        PackDesign {
            cell: CellSpec::lg_m50(),
            series: 4,
            parallel: 2,
            cell_resistance_spread: 0.0,
            group_voltage_spread_v: 0.0,
            interconnect_r_ohm: 0.002,
            contact_r_ohm: 0.004,
            wire: WireRun {
                conductor: Conductor::Mm2(2.0),
                insulation: Insulation::Silicone,
                length_m: 0.5,
                cooling: WireCooling::FreeAir,
                outer_diameter_mm: None,
            },
            fuse: FuseSpec::ato(20),
            bms: None,
            enclosure: None,
            skin: None,
            worn_on_body: true,
        }
    }

    fn load() -> LoadProfile {
        LoadProfile {
            steps: vec![],
            cycles: 1,
            ambient_min_c: 25.0,
            ambient_max_c: 25.0,
            load_v_min: None,
            load_v_max: None,
            charge: None,
        }
    }

    #[test]
    fn terminal_fault_matches_hand_calculation() {
        let f = short_circuit(&pack(), &load(), FaultPoint::Terminals).expect("данные есть");
        // наибольший ток: 16.8 В / (4·24 мОм/2), внешние сопротивления в запас не берутся
        assert!((f.i_max_a - 350.0).abs() < 1e-9, "{}", f.i_max_a);
        // наименьший: 10 В / (2·36 мОм·2 + 2 мОм перемычки + 3.38 мОм предохранитель)
        let hand_min = 10.0 / (2.0 * 0.036 * EOL_RESISTANCE_FACTOR + 0.002 + 0.00338);
        assert!((f.i_min_a - hand_min).abs() < 1e-9, "{}", f.i_min_a);
        assert!(f.fuse_in_path);
    }

    #[test]
    fn load_fault_includes_wire_and_contact() {
        let f = short_circuit(&pack(), &load(), FaultPoint::AtLoad).expect("данные есть");
        // 1 м меди 2 мм² при 20 °C: 1.7241e-8/2e-6 = 8.62 мОм
        let hand_max = 16.8 / (0.048 + 0.0086205);
        assert!((f.i_max_a - hand_max).abs() < 1e-6, "{}", f.i_max_a);
        // при 180 °C: 8.62·(234.5 + 180)/254.5; ячейки: 2·36 мОм·2 (конец службы)
        let r_hot = 0.0086205 * (234.5 + 180.0) / 254.5;
        let r_cells = 2.0 * 0.036 * EOL_RESISTANCE_FACTOR;
        let hand_min = 10.0 / (r_cells + 0.002 + 0.00338 + r_hot + 0.004);
        assert!((f.i_min_a - hand_min).abs() < 1e-3, "{}", f.i_min_a);
        assert!(f.i_min_a < f.i_max_a);
    }

    #[test]
    fn fuse_down_the_wire_does_not_see_terminal_fault() {
        let mut d = pack();
        if let Some(f) = d.fuse.as_mut() {
            f.position_m = 0.2;
        }
        let at_terminals = short_circuit(&d, &load(), FaultPoint::Terminals).expect("есть");
        assert!(!at_terminals.fuse_in_path);
        let after = short_circuit(&d, &load(), FaultPoint::AfterFuse).expect("есть");
        assert!(after.fuse_in_path);
        assert!((after.wire_loop_m - 0.4).abs() < 1e-12);
    }

    #[test]
    fn missing_cell_resistance_gives_no_number() {
        let mut d = pack();
        d.cell.r_low_ohm.value = None;
        assert!(fault_table(&d, &load()).is_empty());
    }
}
