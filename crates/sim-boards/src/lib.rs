//! Каталог плат конструктора, проверка соединений и исполнение машинного кода.

mod arm;
mod avr;
mod catalog;
mod checks;
mod clock;
mod device;
pub mod esp_clock;
pub mod esp_memory;
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
pub use device::{BoardSim, EspBootOptions, EspBootPosition, new_board};
pub use fault::BoardFault;
pub use image::esp_image;
pub use peripheral::esp_adc;
pub use peripheral::esp_dma;
pub use peripheral::esp_flash;
pub use peripheral::esp_rtc::{EspRtc, EspRtcDomains, EspRtcReset};
pub use peripheral::esp_system::EspSystem;
pub use peripheral::esp_system::SystimerPhase;
pub use ports::*;
pub use wiring::{Connection, Outcome, WiringFinding, WiringRule, check_wiring, wiring_outcome};

/// Эталонные сверки блока.
pub fn reference_checks() -> Vec<sim_verify::Check> {
    checks::all()
}
