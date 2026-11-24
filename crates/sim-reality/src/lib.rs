//! Связь стендовых журналов с сигналами и моделями платформы.

mod calibration;
mod capture;
mod identify;
mod profile;
mod replay;

pub use calibration::{Calibration, MeasuredCapture, MeasuredValue, Receipt, Target};
pub use capture::{BuildId, Capture, Channel, Origin, Role, Sampling};
pub use identify::{
    Axis, Criteria, Estimate, Experiment, ExperimentKind, Identification, Outcome, Parameter,
    identify,
};
pub use profile::{Installation, ModelReport, Profile, Report, Simulation, analyze};
pub use replay::{
    AlignedPoint, Alignment, CurveBinding, CurveReport, CurveSource, Gap, InputBinding,
    ReplayReport, ReplayRuntime, replay,
};
