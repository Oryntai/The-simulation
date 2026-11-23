//! Результат компиляции мира. Это контракт между `sim-world` и `sim-runtime`:
//! менять поля можно только согласованно в обоих крейтах.

use std::collections::BTreeMap;

use sim_signal::Rate;

/// Откуда взято число. Оценка не закрывает опасность.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Provenance {
    Datasheet(String),
    Paper(String),
    Measured(String),
    /// Посчитано из других данных, например масса из геометрии и плотности.
    Derived(String),
    Estimate(String),
}

impl Provenance {
    /// Годится ли число для положительного доказательства.
    pub fn can_prove(&self) -> bool {
        !matches!(self, Self::Estimate(_))
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct CompiledBody {
    /// Полное имя в MJCF: `устройство/звено`.
    pub name: String,
    /// Устройство или предмет, которому принадлежит тело.
    pub owner: String,
    pub mass_kg: f64,
    /// Центр масс в кадре тела, м.
    pub com_m: [f64; 3],
    /// Главные моменты инерции относительно центра масс, кг·м².
    pub inertia_kg_m2: [f64; 3],
    pub provenance: Provenance,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CompiledJointKind {
    Hinge,
    Slide,
    Ball,
    Free,
}

#[derive(Clone, Debug, PartialEq)]
pub struct CompiledJoint {
    /// Полное имя в MJCF: `устройство/соединение`.
    pub name: String,
    pub owner: String,
    pub kind: CompiledJointKind,
}

/// Вход обобщённой силы или момента на одну степень свободы соединения.
#[derive(Clone, Debug, PartialEq)]
pub struct EffortChannel {
    /// Уникальное имя канала: `устройство/канал`.
    pub name: String,
    /// Полное имя соединения.
    pub joint: String,
    /// Номер степени свободы внутри соединения (0 у вращательного и поступательного).
    pub dof: usize,
}

/// Привод на входе усилия. Параметры детали читает модель привода по `part`.
#[derive(Clone, Debug, PartialEq)]
pub struct CompiledDrive {
    /// Уникальное имя: `устройство/привод`.
    pub name: String,
    pub owner: String,
    pub part: String,
    /// Полное имя входа усилия.
    pub effort: String,
    /// Полное имя соединения.
    pub joint: String,
    pub params: BTreeMap<String, f64>,
}

/// Датчик с местом установки.
#[derive(Clone, Debug, PartialEq)]
pub struct CompiledSensor {
    /// Уникальное имя: `устройство/датчик` или имя датчика мира.
    pub name: String,
    pub part: String,
    /// Полное имя тела, `None` — закреплён в мире.
    pub body: Option<String>,
    pub position_m: [f64; 3],
    pub quat_wxyz: [f64; 4],
    pub joint: Option<String>,
    /// Полное имя привода.
    pub drive: Option<String>,
    pub params: BTreeMap<String, f64>,
}

/// Упрощение модели, видимое в результате.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Simplification {
    pub subject: String,
    pub text: String,
    /// Существенно для вывода: положительное доказательство по этой теме невозможно.
    pub blocks_proof: bool,
}

/// Мир, готовый к запуску.
#[derive(Clone, Debug, PartialEq)]
pub struct CompiledWorld {
    pub name: String,
    pub seed: u64,
    /// Частота шага физики. Совпадает с `timestep` в `mjcf`.
    pub physics: Rate,
    pub mjcf: String,
    pub bodies: Vec<CompiledBody>,
    pub joints: Vec<CompiledJoint>,
    pub efforts: Vec<EffortChannel>,
    pub drives: Vec<CompiledDrive>,
    pub sensors: Vec<CompiledSensor>,
    pub simplifications: Vec<Simplification>,
}
