//! Ввод схем, статическая проверка и установка живых электрических опытов.
//!
//! В электрическом манифесте формата 1 поле `assemblies.<опыт>.solder` указывает
//! на отдельный файл с `format = 1`, `catalog` и числом интервалов сетки `steps`.
//! Под `instances.<экземпляр>.<стык>` задаются пути `profile` и `series`.
//! Пути разрешаются относительно файла, в котором объявлены.
//!
//! Файл серии формата 1 задаёт `id`, `revision`, `schematic`, `scope`,
//! `connection` и упорядоченные `ends` в виде `компонент/вывод`. В таблице
//! `applicability` задаются `calibration_profile`, измеренный `source.measured`
//! и границы `[min, max]` для `force_n`, `duration_s`, `pitch_m` и `dose_m3`.
//! В таблицах `four_wire_resistance_ohm` и `destructive_strength_n` задаются
//! `range` с измеренным источником и `conditions` замера.

mod input;
mod install;
mod pcb;
pub(crate) mod report;
pub mod solder_input;
pub mod mcu_input;
pub mod emc;

pub use input::{
    CatalogInput, SchematicInput, load_catalog, load_schematic, load_world_schematics,
    parse_catalog, parse_schematic,
};
pub(crate) use install::LiveSummary;
pub use install::{LiveExperiment, LivePath};
