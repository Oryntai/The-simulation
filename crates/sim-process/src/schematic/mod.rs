//! Электрическая схема устройства. Физические участки и выводы сохраняются при раскрытии в MNA.

mod assembly;
mod catalog;
mod checks;
mod compile;
mod graph;
mod modes;
mod pins;
mod reference;

pub use assembly::*;
pub use catalog::*;
pub use checks::*;
pub use compile::*;
pub use graph::*;
pub use modes::*;
pub use pins::*;
pub use reference::*;
pub use sim_circuit;

use crate::ProcessError;

fn invalid(message: impl Into<String>) -> ProcessError {
    ProcessError::InvalidInput(message.into())
}

fn identifier(value: &str) -> bool {
    !value.trim().is_empty() && value.trim() == value && !value.chars().any(char::is_control)
}
