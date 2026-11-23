//! `sim-scenarios` — сценарии мастерской. Каждый сценарий — это модель устройства,
//! его управление и сверки «симуляция ↔ ручной расчёт», которые окно показывает рядом.

pub mod anatomy;
pub mod battery_pack;
pub mod beam;
pub mod blueprint;
pub mod checks;
pub mod circuit_lab;
pub mod constructor;
pub mod device_assess;
pub mod device_catalog;
pub mod device_templates;
pub mod elbow_plant;
pub mod environment;
pub mod exo_anatomy;
pub mod exo_arm;
pub mod exo_io;
pub mod exo_loop;
pub mod free_fall;
pub mod human_limits;
pub mod magnets;
pub mod pcb_board;
pub mod pendulum;
pub mod radio_body;
pub mod room;
pub mod scenario;

pub use scenario::*;

/// Каталог сценариев в порядке показа.
pub fn catalog() -> Vec<Box<dyn Scenario>> {
    vec![
        Box::new(constructor::Constructor::default()),
        Box::new(exo_arm::ExoArm::default()),
        Box::new(exo_anatomy::ExoAnatomy::default()),
        Box::new(exo_loop::ExoLoop::default()),
        Box::new(blueprint::BuildReview::default()),
        Box::new(room::Room::default()),
        Box::new(anatomy::Anatomy::default()),
        Box::new(environment::Environment::default()),
        Box::new(radio_body::RadioBody::default()),
        Box::new(magnets::Magnets::default()),
        Box::new(circuit_lab::CircuitLab::default()),
        Box::new(pcb_board::PcbBoard::default()),
        Box::new(beam::BeamStrut::default()),
        Box::new(pendulum::Pendulum::default()),
        Box::new(free_fall::FreeFall::default()),
    ]
}
