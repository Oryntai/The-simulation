//! `sim-battery` — безопасность батареи: короткое замыкание, провода, предохранители,
//! BMS, перегрев и порог теплового разгона.
//!
//! Модули:
//! * [`cells`] — каталог ячеек, у каждого числа свой источник;
//! * [`pack`] — сборка S×P и профиль нагрузки (простые поля для формы);
//! * [`wire`] — AWG и мм², допустимый ток, падение напряжения, адиабата IEC 60949, Ондердонк;
//! * [`protection`] — предохранитель (время-токовая характеристика, I²t) и BMS с MOSFET;
//! * [`fault`] — ток КЗ в выбранной точке, наибольший и наименьший;
//! * [`thermal`] — нагрев ячеек в корпусе и температура поверхности у кожи;
//! * [`assess`] — единая оценка [`assess_pack`] для вердикта;
//! * [`checks`] — эталонные сверки вторым независимым способом.
//!
//! Единицы — СИ, если в имени не сказано иное (`_c` — °C, `_mm2` — мм², `_mv` — мВ).
//!
//! Главное правило модуля: ложное «годно» на опасной сборке хуже ложного «негодно». Поэтому
//! `Pass` — только при конечных числах и выполненном пределе, некорректный вход — `Fail`,
//! отсутствие данных — `NotAnalyzed`, а числа без источника взяты в опасную сторону.

pub mod assess;
pub mod cells;
pub mod checks;
pub mod fault;
pub mod pack;
pub mod protection;
pub mod thermal;
pub mod wire;

pub use assess::{
    ALL_CHECKS, BatteryCheck, BatteryFinding, Outcome, assess_pack, overall, validate,
};
pub use cells::{
    CellFormat, CellSpec, ChargeLimit, Chemistry, LipoInput, Param, RunawayThreshold, SourceKind,
    catalog, find_cell,
};
pub use fault::{ALL_FAULT_POINTS, FaultCurrents, FaultPoint, fault_table, short_circuit};
pub use pack::{ChargeProfile, LoadProfile, LoadStep, PackDesign, PackSummary};
pub use protection::{BmsSpec, Bq769x0Config, FuseSpec, MosfetSpec, TccPoint};
pub use sim_env::contact::ContactClass;
pub use thermal::{EnclosureDesign, SkinInterface, ThermalResult, simulate_temperature};
pub use wire::{Conductor, Insulation, WireCooling, WireRun};

use sim_verify::Check;

/// Эталонные сверки блока.
pub fn reference_checks() -> Vec<Check> {
    checks::all()
}
