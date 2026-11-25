//! Электрические свойства физических контактов и модель сброса на общей шкале времени.

mod pins;
mod rail;
mod step;
mod transient;

pub use pins::*;
pub(crate) use step::Electrical;
pub use step::{
    BoardConnection, BoardCurrents, CircuitModel, ElectricalProbe, ElectricalSetup, GpioConnection,
    LoadConnection, PowerPhase, SourceStep,
};
pub use transient::{LivePin, TransientCircuit};
