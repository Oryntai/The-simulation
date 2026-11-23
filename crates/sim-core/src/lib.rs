//! `sim-core` — ядро симулятора мастерской устройств.
//!
//! * [`spatial`] — 6D-алгебра Featherstone (преобразования Плюккера, пространственная инерция);
//! * [`model`] — дерево тел и шарниров (аналог MJCF/URDF), геометрия для показа;
//! * [`dynamics`] — RNEA, ABA, матрица масс, энергия, импульс;
//! * [`integrate`] — шаг по времени (явный/полунеявный Эйлер, RK4).
//!
//! Без зависимостей от графики: ядро тестируется и запускается без окна.

pub mod dynamics;
pub mod integrate;
pub mod model;
pub mod spatial;

pub use integrate::{Inputs, Integrator, PointForce, State, step};
pub use model::{Body, G_STANDARD, Joint, Model, Pose, Shape, Visual};
/// Общий генератор платформы для детерминированных сверок движков.
pub use sim_signal::Rng;
pub use spatial::{Inertia, Mat3, SVec, Vec3, Xform};
