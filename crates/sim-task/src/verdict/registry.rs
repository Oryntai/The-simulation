//! Реестр обязательств. Идентификаторы являются частью формата отчёта.

use serde::Serialize;
use sim_process::work::HazardKind;

use super::{ComponentKind, Group};

pub const REGISTRY_VERSION: u32 = 1;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Method {
    Section,
    Dynamics,
    Process,
    Interaction,
    Control,
    Measurement,
    Inventory,
}

#[derive(Clone, Copy, Debug, Serialize)]
pub struct ClassSpec {
    pub class: HazardClass,
    pub id: &'static str,
    pub group: Group,
    pub applies_to: &'static [ComponentKind],
    pub method: Method,
    pub limit_origin: &'static str,
    pub independent_check: &'static str,
}

macro_rules! registry {
    ($($class:ident, $id:literal, $group:ident, [$($kind:ident),*], $method:ident, $limit:literal, $check:literal;)*) => {
        #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
        pub enum HazardClass { $(#[serde(rename = $id)] $class,)* }

        pub const REGISTRY: &[ClassSpec] = &[$(ClassSpec {
            class: HazardClass::$class, id: $id, group: Group::$group,
            applies_to: &[$(ComponentKind::$kind),*], method: Method::$method,
            limit_origin: $limit, independent_check: $check,
        },)*];

        impl HazardClass {
            pub fn spec(self) -> &'static ClassSpec {
                // Индексы перечисления и таблицы порождаются одной декларацией.
                &REGISTRY[self as usize]
            }
        }
    };
}

registry! {
    StaticStrength, "static_strength", Mechanics, [Part], Section, "расчётный предел материала в условиях нагрузки", "равновесие и модуль сечения";
    Fatigue, "fatigue", Parts, [CyclicLoad], Dynamics, "кривая усталости и концентрация", "цикл нагрузки и независимый расчёт либо стенд";
    Retention, "retention", Mechanics, [Retention], Dynamics, "допустимый сдвиг или усилие удержания", "баланс сил и контрольная нагрузка";
    ConnectionFailure, "connection_failure", Parts, [Connection], Process, "огибающая прочности соединения", "реакция опоры и баланс сил и моментов";
    TravelJam, "travel_jam", Mechanics, [MotionLimit], Dynamics, "предел перемещения и допустимая реакция", "геометрия зазора и межсеточное сравнение";
    Fall, "fall", Mechanics, [FreeSupport], Dynamics, "граница устойчивости или событие падения", "опорный многоугольник или баланс момента";
    ThreadFailure, "thread_failure", Parts, [], Process, "огибающая нагрузки резьбы", "виртуальная работа, осевая нагрузка и момент";
    Impact, "impact", Contact, [ImpactPath], Process, "применимый предел силы или импульса", "баланс импульса и межсеточное сравнение";
    Pinch, "pinch", Contact, [PinchPath], Process, "геометрия зазора и предел сдавливания или сдвига", "кинематика, реакции и межсеточное сравнение";
    SharpEdge, "sharp_edge", Contact, [SharpSurface], Process, "применимая геометрия и предел контакта", "обмер кромки и анализ достижимости";
    TissueLoad, "tissue_load", Human, [ParticipantContact], Interaction, "тканевый предел по области, площади, длительности и популяции", "баланс сил и межсеточное сравнение";
    ContactStructure, "contact_structure", Contact, [ParticipantContact], Interaction, "допустимая реакция детали и опоры", "баланс сил и межсеточное сравнение";
    DangerousMotion, "dangerous_motion", Human, [MotionAtParticipant], Dynamics, "предел скорости, ускорения или рывка для режима", "кинематика и межсеточное сравнение";
    Burns, "burns", Human, [HeatPath], Process, "применимый порог теплового воздействия", "тепловой баланс и независимая модель воздействия";
    DriveOverload, "drive_overload", Actuators, [Drive], Dynamics, "паспортная или измеренная огибающая привода", "баланс энергии и стенд";
    WiringHeat, "wiring_heat", Wiring, [Wire], Dynamics, "токовая и тепловая огибающая проводки", "баланс Джоуля и стенд";
    Overcurrent, "overcurrent", Power, [Supply, Wire], Process, "предел тока и время срабатывания защиты", "законы Кирхгофа и срабатывание защиты";
    Battery, "battery", Battery, [Battery], Dynamics, "паспортные пределы и огибающая защиты", "баланс заряда и нагрузочный стенд";
    ElectricShock, "electric_shock", Power, [ExposedElectricalPath], Process, "применимый предел доступного напряжения и воздействия", "схема, измерение тока и доступности";
    StoredEnergy, "stored_energy", Power, [EnergyPath, Drive, Battery, Supply], Process, "предел доступной энергии для пути и окна", "баланс запасённой и переданной энергии";
    Ingress, "ingress", Power, [EnvironmentalPath], Process, "порог воздействия среды и запретное событие", "независимое измерение проникновения или его эффекта";
    SensorFault, "sensor_fault", Wiring, [Sensor], Control, "обязательная реакция на отказ датчика", "подтверждение инъекции и принятой команды";
    WireFault, "wire_fault", Wiring, [Wire], Control, "обязательная реакция на разрыв канала", "подтверждение инъекции и принятой команды";
    LinkFault, "link_fault", Firmware, [Communication], Control, "предел просрочки и обязательная реакция при потере связи", "номера и времена доставок и принятой команды";
    ControllerHalt, "controller_halt", Firmware, [Controller], Control, "обязательная реакция при остановке контроллера", "запись остановки и реакция исполнительной модели";
    OutputFault, "output_fault", Actuators, [Output, Supply, Drive], Control, "обязательная реакция при отказе выхода или питания", "запись инъекции и реакция исполнительной модели";
    EmergencyStop, "emergency_stop", Firmware, [Protection, Drive], Control, "время отключения и предел остаточного воздействия", "реальное принятие команды и независимая проверка энергии";
    Measurement, "measurement", Measurements, [CriticalMeasurement], Measurement, "допустимое расхождение для версии и режима", "привязанный стендовый ряд и модель";
    ProgramIdentity, "program_identity", Firmware, [Program], Control, "отпечаток исполняемого образа и путь команды", "идентичность установленного образа и журнал платы";
    TaskCriterion, "task_criterion", Measurements, [], Dynamics, "граница постановки", "истинный ряд в объявленном окне";
    Execution, "execution", Measurements, [], Dynamics, "полнота живого прогона и установки", "повтор прогона и совпадение журналов";
    Inventory, "inventory", Parts, [], Inventory, "полнота состава и области", "разрешённый граф и исходные документы";
}

impl From<HazardKind> for HazardClass {
    fn from(kind: HazardKind) -> Self {
        match kind {
            HazardKind::Burns => Self::Burns,
            HazardKind::Impact => Self::Impact,
            HazardKind::Pinch => Self::Pinch,
            HazardKind::ThreadFailure => Self::ThreadFailure,
            HazardKind::ElectricShock => Self::ElectricShock,
            HazardKind::Overcurrent => Self::Overcurrent,
            HazardKind::StoredEnergy => Self::StoredEnergy,
            HazardKind::SharpEdge => Self::SharpEdge,
            HazardKind::Ingress => Self::Ingress,
            HazardKind::Retention => Self::Retention,
            HazardKind::ConnectionFailure => Self::ConnectionFailure,
            HazardKind::ContactStructure => Self::ContactStructure,
            HazardKind::DangerousMotion => Self::DangerousMotion,
            HazardKind::Fatigue => Self::Fatigue,
        }
    }
}
