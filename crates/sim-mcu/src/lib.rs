//! `sim-mcu` — виртуальный микроконтроллер VMCU-C3: ядро RV32IMC + Zicsr (как у
//! ESP32-C3, 160 МГц) и своя простая периферия. Исполняет настоящие прошивки,
//! собранные `rustc` под `riscv32imc-unknown-none-elf` (каталог `firmware/`).
//!
//! * [`Mcu`] — машина и интерфейс хоста: загрузка ELF, шаг по времени, входы/выходы.
//! * [`exo`] — контракт прошивки контроллера локтевого экзоскелета `exo-ctrl`:
//!   блок конфигурации, радиопротокол, шкалы датчиков.
//! * [`bench`] — простой стенд-объект (предплечье + DC-мотор) для замкнутых тестов.
//! * [`reference_checks`] — сверки со вторым независимым способом.
//!
//! Карта памяти и регистры — `crates/sim-mcu/MEMORY_MAP.md`.

pub mod bench;
pub mod bus;
pub mod checks;
pub mod cpu;
pub mod elf;
pub mod exo;
pub mod firmware;
pub mod llvm_vectors;
pub mod mcu;
pub mod periph;

pub use mcu::{Fault, FaultKind, FaultSource, HaltReason, Mcu, McuError, ResetReason, RunReport};

/// Все сверки этого блока с эталоном (вторым независимым способом): см. [`checks`].
pub fn reference_checks() -> Vec<sim_verify::Check> {
    checks::all()
}
