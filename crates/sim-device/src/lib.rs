//! `sim-device` — конструктор устройств: из чего собрано устройство, как оно нагружено и
//! что будет с телом человека рядом с ним.
//!
//! * [`model`] — описание устройства: звенья, шарниры, материалы, электроника, условия;
//! * [`file`] — файл проекта `.device`, читается и правится руками;
//! * [`mechanics`] — масса, худшие моменты в шарнирах и изгиб звеньев;
//! * [`safety`] — удар, зажим и зазоры от раздавливания по участкам тела;
//! * [`measure`] — замеры на настоящем устройстве против предсказания симуляции.

pub mod file;
pub mod measure;
pub mod mechanics;
pub mod model;
pub mod safety;
pub mod settings;
pub mod wiring;

pub use mechanics::{ActuatorData, Analysis, Catalog, JointLoad, LinkLoad, analyze};
pub use model::*;
pub use settings::*;
pub use wiring::Wiring;
