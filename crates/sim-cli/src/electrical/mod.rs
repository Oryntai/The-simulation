//! Ввод схем, статическая проверка и установка живых электрических опытов.

mod input;
mod install;
pub(crate) mod report;

pub use input::{
    CatalogInput, SchematicInput, load_catalog, load_schematic, load_world_schematics,
    parse_catalog, parse_schematic,
};
pub(crate) use install::LiveSummary;
pub use install::{LiveExperiment, LivePath};
