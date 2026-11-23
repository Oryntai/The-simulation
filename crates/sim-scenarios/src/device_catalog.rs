//! Каталожные идентификаторы и общий адаптер для расчёта и конструктора.

use sim_boards::BoardId;
use sim_device::{ActuatorData, Catalog};
use sim_materials::{MaterialClass, MaterialId};

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PartOption {
    pub id: String,
    pub title: String,
}

impl PartOption {
    fn new(id: &str, title: &str) -> Self {
        Self {
            id: id.into(),
            title: title.into(),
        }
    }
}

fn material_key(m: MaterialId) -> &'static str {
    use MaterialId::*;
    match m {
        Pla => "pla",
        Petg => "petg",
        Abs => "abs",
        Asa => "asa",
        Pa12 => "pa12",
        PaCf => "pa-cf",
        Tpu95a => "tpu95a",
        Al6061T6 => "al6061-t6",
        Al7075T6 => "al7075-t6",
        SteelS235 => "s235",
        Stainless304 => "inox304",
        Ti6Al4V => "ti6al4v",
        CfrpTube => "cfrp-tube",
    }
}

pub fn material(id: &str) -> Option<MaterialId> {
    MaterialId::ALL
        .into_iter()
        .find(|m| material_key(*m) == id || m.material().name == id)
}

pub fn board(id: &str) -> Option<BoardId> {
    sim_boards::BoardCatalog::builtin()
        .ok()?
        .get(id)
        .map(|entry| entry.profile)
        .filter(|profile| *profile != BoardId::Virtual)
}

pub fn materials() -> Vec<PartOption> {
    MaterialId::ALL
        .into_iter()
        .map(|m| PartOption::new(material_key(m), m.material().name))
        .collect()
}

pub fn actuators() -> Vec<PartOption> {
    sim_actuators::actuators()
        .iter()
        .map(|s| PartOption::new(s.id, s.name))
        .collect()
}

pub fn boards() -> Vec<PartOption> {
    sim_boards::BoardCatalog::builtin()
        .expect("встроенный каталог плат проверяется тестами")
        .entries()
        .iter()
        .filter(|entry| entry.profile != BoardId::Virtual)
        .map(|entry| PartOption::new(&entry.id, &entry.name))
        .collect()
}

pub fn drivers() -> Vec<PartOption> {
    sim_actuators::external_drivers()
        .into_iter()
        .map(|s| PartOption::new(s.id, s.name))
        .collect()
}

pub fn regulators() -> Vec<PartOption> {
    sim_actuators::regulators()
        .iter()
        .map(|s| PartOption::new(s.id, s.name))
        .collect()
}

pub fn cells() -> Vec<PartOption> {
    sim_battery::catalog()
        .into_iter()
        .map(|s| PartOption::new(&s.name, &s.name))
        .collect()
}

pub fn sensors() -> Vec<PartOption> {
    sim_actuators::sensors()
        .iter()
        .map(|s| PartOption::new(s.id, s.name))
        .collect()
}

pub fn is_printable(id: &str) -> bool {
    material(id).is_some_and(|m| m.material().class == MaterialClass::Printed)
}

#[derive(Clone, Copy, Debug, Default)]
pub struct WorkshopCatalog;

impl Catalog for WorkshopCatalog {
    fn density_kg_m3(&self, material_id: &str) -> Option<f64> {
        material(material_id).map(|id| id.material().density)
    }

    fn actuator(&self, id: &str) -> Option<ActuatorData> {
        let a = sim_actuators::actuator_by_id(id)?;
        let reflected_inertia_kg_m2 = if [
            a.rotor_inertia_kg_m2,
            a.gear_input_inertia_kg_m2,
            a.gear_ratio,
        ]
        .into_iter()
        .all(|s| s.is_proof())
        {
            a.reflected_inertia_kg_m2()
        } else {
            None
        };
        Some(ActuatorData {
            mass_kg: a.mass_kg.get()?,
            reflected_inertia_kg_m2,
            peak_torque_nm: a.peak_torque_nm.get(),
        })
    }
}
