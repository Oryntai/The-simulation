//! `sim-materials` — материалы конструктора: печатные пластики, металлы, усталость,
//! проверка «можно ли такую деталь напечатать».
//!
//! * [`catalog`] — каталог материалов с источником каждого значения;
//! * [`section`] — сечения, напряжения, потеря устойчивости (Эйлер, Джонсон);
//! * [`printed`] — прочность напечатанной детали: слои, стенки, заполнение, температура;
//! * [`fatigue`] — S-N (Баскин), Гудман, концентраторы Петерсона, надрез, Майнер;
//! * [`fasteners`] — болты M2–M8, вырыв резьбы и вставок, смятие;
//! * [`printability`] — проверка печати: блокеры и предупреждения;
//! * [`assess`] — единая оценка детали [`assess_part`] для вердикта;
//! * [`checks`] — эталонные сверки вторым независимым способом.
//!
//! Принтер как устройство не моделируется: пластик здесь — обычный материал детали.
//! Главное правило модуля: `NotAnalyzed` — это не «годно», а NaN и бесконечность — всегда `Fail`.

pub mod assess;
pub mod catalog;
pub mod checks;
pub mod fasteners;
pub mod fatigue;
pub mod printability;
pub mod printed;
pub mod section;

pub use assess::{
    BODY_WORN_MIN_MARGIN, Fastening, FindingKind, HoleOrientation, LoadBlock, LoadCase, Margins,
    MeasuredSn, Outcome, PartFinding, PartSpec, PrintSpec, StressRaiser, assess_part, overall,
};
pub use catalog::{FatigueData, Material, MaterialClass, MaterialId, temperature_factor};
pub use fasteners::{BoltGrade, BoltSize, FastenerSpec, ThreadEngagement};
pub use fatigue::{Reliability, SnCurve, Surface};
pub use printability::{PrintGeometry, PrintReport, check_printability};
pub use printed::{PrintProcess, PrintedStrength, printed_strength};
pub use section::{EndCondition, Section};

use sim_verify::Check;

/// Эталонные сверки блока.
pub fn reference_checks() -> Vec<Check> {
    checks::all()
}
