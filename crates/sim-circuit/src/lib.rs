//! `sim-circuit` — Электрические схемы: модифицированный узловой анализ (MNA), разбор SPICE-подмножества, аккумулятор, расчёты печатных плат.
//!
//! * [`netlist`] — разбор нетлиста (подмножество SPICE, подсхемы раскрываются);
//! * [`circuit`] — схема, готовая к расчёту ([`Circuit`]);
//! * [`mna`] — сборка MNA, метод Ньютона, рабочая точка ([`operating_point`]);
//! * [`transient`] — переходный процесс: пошаговый [`Transient`] для совместного
//!   моделирования и пакетный [`run_tran`];
//! * [`examples`] — готовые схемы для редактора нетлистов;
//! * [`battery`] — модель Li-ion ячейки LG INR21700-M50 (эквивалентная схема) и батареи;
//! * [`pcb`] — дорожки, переходные отверстия, микрополосок, нагрев компонентов, плата;
//! * [`checks`] — сверки с эталоном (вторым независимым способом), в т.ч. с ngspice.

pub mod adc;
pub mod battery;
pub mod checks;
pub mod circuit;
pub mod devices;
pub mod emc;
pub mod error;
pub mod examples;
pub mod mna;
pub mod netlist;
pub mod ngspice;
pub mod pcb;
pub mod transient;
pub mod units;
mod waveform;

pub use circuit::Circuit;
pub use error::SimError;
pub use mna::{OperatingPoint, SimOptions, operating_point, operating_point_with};
pub use netlist::{Netlist, ParseError, parse};
pub use transient::{TranOptions, TranStats, Transient, Waveforms, run_tran, run_tran_with};

/// Все сверки этого блока с эталоном (вторым независимым способом).
///
/// Если найден ngspice (`assets/third_party/ngspice/bin/ngspice` или переменная `NGSPICE`),
/// добавляются и сверки с ним; иначе они молча пропускаются.
pub fn reference_checks() -> Vec<sim_verify::Check> {
    let mut v = checks::circuit::all();
    v.extend(checks::battery::all());
    v.extend(checks::pcb::all());
    if let Some(ng) = ngspice::find_ngspice() {
        v.extend(checks::ngspice_cmp::all(&ng));
    }
    v
}
