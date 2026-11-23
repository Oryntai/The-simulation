//! `sim-actuators` — каталог приводов, драйверов, датчиков и стабилизаторов для конструктора
//! и проверка, выдержит ли выбранная деталь свою нагрузку.
//!
//! Модули:
//! * [`spec`] — паспортное число с происхождением: паспорт, продавец, замер, расчёт, оценка;
//! * [`actuator`] — привод и его прямая «момент–скорость» при заданном напряжении;
//! * [`driver`], [`regulator`], [`sensor`] — остальные детали каталога;
//! * [`assess`] — проверка привода на цикле работы ([`assess_actuator`]);
//! * [`power`] — сводка питания устройства ([`power_budget`]);
//! * [`thermal`] — установившийся тепловой цикл обмотки;
//! * [`checks`] — эталонные сверки и найденные противоречия в паспортах.
//!
//! Главное правило: ложное «годится» хуже ложного «не годится». Поэтому:
//! при неполных данных — `NotAnalyzed`. Оценка и сведения продавца не доказывают `Pass`;
//! NaN и бесконечность — всегда `Fail`; из расходящихся паспортов берётся безопасное число.
//!
//! ```
//! use sim_actuators::{DutyCycle, Outcome, actuator_by_id, assess_actuator, overall};
//! let sg90 = actuator_by_id("sg90").unwrap();
//! // Условная нагрузка для примера: 1 кг на плече 20 см, момент по M = m·g·l.
//! let torque_nm = 1.0 * 9.80665 * 0.20;
//! let duty = DutyCycle::new(4.8, 25.0).segment(torque_nm, 0.0, 10.0).hold_unpowered(0.0);
//! assert_eq!(overall(&assess_actuator(sg90, None, &duty)), Outcome::Fail);
//! ```
//!
//! Единицы — СИ (Н·м, рад/с, А, В, Вт, с), температуры — °C.

pub mod actuator;
pub mod assess;
mod catalog_actuators;
pub mod checks;
pub mod driver;
pub mod power;
pub mod regulator;
pub mod sensor;
pub mod spec;
pub mod thermal;

use std::sync::OnceLock;

pub use actuator::{
    Actuator, ActuatorKind, Backdrive, Brake, Magnet, MotorLine, MotorType, VoltagePoint,
};
pub use assess::{
    ActuatorCheck, ActuatorFinding, DutyCycle, DutySegment, ElectricalLoad, Outcome,
    assess_actuator, electrical_load, overall,
};
pub use checks::{datasheet_discrepancies, reference_checks};
pub use driver::{Driver, DriverKind};
pub use power::{
    ActuatorLoad, Battery, BoardLoad, PowerBudget, PowerCheck, PowerFinding, PowerInput, PowerLine,
    RailInput, RailSummary, SensorLoad, power_budget,
};
pub use regulator::{Regulator, RegulatorKind, RegulatorOperation};
pub use sensor::{Sensor, SensorKind};
pub use spec::{KGF_CM, Provenance, RPM, Spec};

/// Все приводы каталога.
pub fn actuators() -> &'static [Actuator] {
    static CATALOG: OnceLock<Vec<Actuator>> = OnceLock::new();
    CATALOG.get_or_init(catalog_actuators::build)
}

/// Все драйверы каталога, включая встроенные.
pub fn drivers() -> &'static [Driver] {
    static CATALOG: OnceLock<Vec<Driver>> = OnceLock::new();
    CATALOG.get_or_init(driver::build)
}

/// Драйверы, которые можно выбрать отдельно (без встроенных в приводы).
pub fn external_drivers() -> Vec<&'static Driver> {
    drivers()
        .iter()
        .filter(|d| d.kind != DriverKind::Integrated)
        .collect()
}

/// Все стабилизаторы каталога.
pub fn regulators() -> &'static [Regulator] {
    static CATALOG: OnceLock<Vec<Regulator>> = OnceLock::new();
    CATALOG.get_or_init(regulator::build)
}

/// Все датчики каталога.
pub fn sensors() -> &'static [Sensor] {
    static CATALOG: OnceLock<Vec<Sensor>> = OnceLock::new();
    CATALOG.get_or_init(sensor::build)
}

/// Привод по id.
pub fn actuator_by_id(id: &str) -> Option<&'static Actuator> {
    actuators().iter().find(|a| a.id == id)
}

/// Драйвер по id.
pub fn driver_by_id(id: &str) -> Option<&'static Driver> {
    drivers().iter().find(|d| d.id == id)
}

/// Стабилизатор по id.
pub fn regulator_by_id(id: &str) -> Option<&'static Regulator> {
    regulators().iter().find(|r| r.id == id)
}

/// Датчик по id.
pub fn sensor_by_id(id: &str) -> Option<&'static Sensor> {
    sensors().iter().find(|s| s.id == id)
}
