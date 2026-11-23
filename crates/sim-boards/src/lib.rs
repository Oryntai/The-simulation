//! Каталог плат конструктора, проверка соединений и исполнение машинного кода.

mod arm;
mod avr;
mod catalog;
mod checks;
mod clock;
mod device;
mod fault;
mod image;
mod memory;
mod peripheral;
mod ports;
mod riscv;
mod virtual_board;
mod wiring;

pub use catalog::{
    BoardCatalog, BoardEntry, BoardId, BoardSpec, PinSpec, Range, SupplyRail, board_pins,
    board_spec, pin_spec, supply_range,
};
pub use device::{BoardSim, new_board};
pub use fault::BoardFault;
pub use ports::*;
pub use wiring::{Connection, Outcome, WiringFinding, WiringRule, check_wiring, wiring_outcome};

/// Эталонные сверки блока.
pub fn reference_checks() -> Vec<sim_verify::Check> {
    checks::all()
}
