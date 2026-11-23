//! `sim-human` — человек под нагрузкой: мышцы плеча, локтя и спины, динамика подъёма,
//! отказ привода, усталость и разброс между людьми.

mod assessment;
mod data;
mod failure;
mod fatigue;
mod lift;
mod load;
mod population;
mod reference;
mod rig;
mod solver;

pub mod scene;

pub use assessment::{
    DriveFailureReport, HumanCheck, HumanFinding, HumanLoadCase, HumanReport, LiftInput,
    LiftReport, LoadPoint, MuscleLoad, Outcome, ReactionPeaks, analyze_human, assess_human,
};
pub use failure::FailureOutcome;
pub use fatigue::{EnduranceVerdict, FatigueJoint};
pub use load::Strengths;
pub use population::{JointAction, Person, Sex, StrengthEstimate, measured_strength};
pub use reference::{known_discrepancies, reference_checks};
pub use rig::{BodyPose, C_ELBOW, C_ELEV, C_L5_AR, C_L5_FE, C_L5_LB, C_PLANE, C_PROSUP, C_ROT, NC};
