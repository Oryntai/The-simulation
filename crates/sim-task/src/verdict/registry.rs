//! Реестр обязательств. Идентификаторы являются частью формата отчёта.

use serde::{Deserialize, Serialize};
use sim_process::{
    schematic::{
        ElectricalClass, capacitor_stress::CAPACITOR_STRESS_CODE,
        deep_discharge::DEEP_DISCHARGE_CODE, fuse_coordination::FUSE_COORDINATION_CODE,
        sleep_drain::SLEEP_DRAIN_CODE, switch_thermal::SWITCH_THERMAL_CODE,
    },
    work::{HazardKind, cable_flex::CABLE_FLEX_CODE},
};

use super::{ComponentKind, Group};

pub const REGISTRY_VERSION: u32 = 3;

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
    Electrical,
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

macro_rules! class_spec {
    ($class:ident, $id:literal, $group:ident, [$($kind:ident),*], $method:ident, $limit:literal, $check:literal) => {
        ClassSpec {
            class: HazardClass::$class, id: $id, group: Group::$group,
            applies_to: &[$(ComponentKind::$kind),*], method: Method::$method,
            limit_origin: $limit, independent_check: $check,
        }
    };
}

macro_rules! registry {
    (
        $($class:ident, $id:literal, $group:ident, [$($kind:ident),*], $method:ident, $limit:literal, $check:literal;)*
        @electrical {
            $($electric:ident, $code:literal, $electric_group:ident, $origin:literal, $verification:literal;)*
        }
    ) => {
        #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Deserialize, Serialize)]
        pub enum HazardClass {
            $(#[serde(rename = $id)] $class,)*
            $(#[serde(rename = $code)] $electric,)*
        }

        /// Реестр классов общего корпуса этапа 6.
        pub const REGISTRY: &[ClassSpec] = &[$(
            class_spec!($class, $id, $group, [$($kind),*], $method, $limit, $check),
        )*];

        /// Дополнительные классы электрического вердикта.
        pub const ELECTRICAL_REGISTRY: &[ClassSpec] = &[$(
            class_spec!($electric, $code, $electric_group, [], Electrical, $origin, $verification),
        )*];

        impl HazardClass {
            pub fn spec(self) -> &'static ClassSpec {
                let index = self as usize;
                if index < REGISTRY.len() {
                    &REGISTRY[index]
                } else {
                    &ELECTRICAL_REGISTRY[index - REGISTRY.len()]
                }
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
    ThreadFailure, "thread_failure", Parts, [ThreadedDrive], Process, "огибающая нагрузки резьбы", "виртуальная работа, осевая нагрузка и момент";
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
    @electrical {
        AssemblyMismatch, "assembly_mismatch", Wiring, "сопряжение каталожных контактов", "таблица сопряжений и сверка монтажа";
        MissingGround, "missing_ground", Wiring, "штатный путь возврата сигнала", "прозвонка контура и проверка обрыва";
        ReversePolarity, "reverse_polarity", Power, "обратный предел полярной детали", "знак напряжения на клеммах";
        PinOvervoltage, "pin_overvoltage", Wiring, "предел вывода в каждом состоянии питания", "напряжение относительно шин и ток инжекции";
        GroundSwitchBackfeed, "ground_switch_backfeed", Wiring, "ток обратного пути и остаточное питание", "ток через сигнальный провод";
        SourceConflict, "source_conflict", Power, "предел обратного тока источника", "разность ЭДС и сопротивление контура";
        RailSag, "rail_sag", Power, "порог и выдержка сброса", "падение напряжения, баланс заряда и событие сброса";
        StartupActuation, "startup_actuation", Firmware, "штатное состояние нагрузки при пуске", "подтяжки, ток и реакция нагрузки";
        InductiveSpike, "inductive_spike", Power, "импульсный предел", "напряжение ограничения и сходимость по шагу";
        SwitchUnderdrive, "switch_underdrive", Power, "применимая область ключа", "напряжение управления и баланс потерь";
        CapacitorStress, "capacitor_stress", Power, "пределы напряжения, RMS пульсаций и температуры", "пик напряжения, интеграл квадрата тока и баланс потерь";
        SwitchThermal, "switch_thermal", Power, "тепловая характеристика монтажа, предел Tj и линейная SOA", "стационарный тепловой баланс и переходная тепловая модель";
        EsdEvent, "esd_event", Wiring, "импульсные пределы напряжения и тока защищаемого вывода", "остаточные U/I на адресованных клеммах и сопоставление со стендовым воздействием";
        EmiReset, "emi_reset", Firmware, "пороги и выдержки входа сброса в заявленном монтаже", "синхронная запись напряжения входа, сброса и доставки в опыте и контроле";
        SolderDefect, "solder_defect", Wiring, "окно пайки по температуре, времени и объёму для данного стыка", "аналитический тепловой баланс, четырёхпроводный замер сопротивления и разрушающее испытание прочности";
        CreepageClearance, "creepage_clearance", Wiring, "пределы воздушного зазора и пути утечки для материала, напряжения и среды", "обмер меди и поверхности по той же ревизии";
        GroundBounce, "ground_bounce", Wiring, "допустимое смещение опоры приёмника для тока и фронта режима", "расчёт RI + L·di/dt и синхронный дифференциальный замер";
    }
}

impl HazardClass {
    pub(super) fn pcb_measurement(self) -> Option<&'static str> {
        match self {
            Self::CreepageClearance => Some("нужны измеренные зазоры и пути утечки, данные материала, напряжения и среды для ревизии платы"),
            Self::GroundBounce => Some("нужен замер смещения опоры на входе приёмника при огибающей тока и фронтов этой ревизии"),
            _ => None,
        }
    }

    pub(super) fn emc_measurement(self) -> Option<&'static str> {
        match self {
            Self::EsdEvent => Some("нужен адресованный контактный разряд со стендовым замером остаточных U/I"),
            Self::EmiReset => Some("нужен связанный опыт и контроль с синхронным замером напряжения входа, сброса и доставки"),
            _ => None,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct DepthCauseSpec {
    pub code: &'static str,
    pub class: HazardClass,
    /// Смежные классы со своими обязательствами и условиями доказательства.
    pub related: &'static [HazardClass],
    pub independent_check: &'static str,
}

pub const DEPTH_REGISTRY: &[DepthCauseSpec] = &[
    DepthCauseSpec {
        code: CAPACITOR_STRESS_CODE,
        class: HazardClass::CapacitorStress,
        related: &[HazardClass::InductiveSpike],
        independent_check: "пик напряжения, интеграл квадрата тока и баланс потерь",
    },
    DepthCauseSpec {
        code: FUSE_COORDINATION_CODE,
        class: HazardClass::Overcurrent,
        related: &[HazardClass::WiringHeat, HazardClass::SwitchThermal],
        independent_check: "полная TCC, интеграл I²t и время повреждения ключа по SOA",
    },
    DepthCauseSpec {
        code: SWITCH_THERMAL_CODE,
        class: HazardClass::SwitchThermal,
        related: &[HazardClass::SwitchUnderdrive],
        independent_check: "стационарный тепловой баланс и переходная тепловая модель",
    },
    DepthCauseSpec {
        code: DEEP_DISCHARGE_CODE,
        class: HazardClass::Battery,
        related: &[],
        independent_check: "напряжение каждой ячейки под нагрузкой, баланс заряда и момент отсечки",
    },
    DepthCauseSpec {
        code: SLEEP_DRAIN_CODE,
        class: HazardClass::Battery,
        related: &[],
        independent_check: "интеграл токов покоя и пробуждения, доступный заряд и порог отсечки",
    },
    DepthCauseSpec {
        code: CABLE_FLEX_CODE,
        class: HazardClass::ConnectionFailure,
        related: &[HazardClass::Fatigue, HazardClass::WireFault],
        independent_check: "длина и радиус на траектории, шаблон монтажа и испытание на циклы",
    },
];

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

impl From<ElectricalClass> for HazardClass {
    fn from(class: ElectricalClass) -> Self {
        match class {
            ElectricalClass::AssemblyMismatch => Self::AssemblyMismatch,
            ElectricalClass::MissingGround => Self::MissingGround,
            ElectricalClass::ReversePolarity => Self::ReversePolarity,
            ElectricalClass::PinOvervoltage => Self::PinOvervoltage,
            ElectricalClass::GroundSwitchBackfeed => Self::GroundSwitchBackfeed,
            ElectricalClass::SourceConflict => Self::SourceConflict,
            ElectricalClass::RailSag => Self::RailSag,
            ElectricalClass::StartupActuation => Self::StartupActuation,
            ElectricalClass::InductiveSpike => Self::InductiveSpike,
            ElectricalClass::SwitchUnderdrive => Self::SwitchUnderdrive,
            ElectricalClass::ConnectionOvercurrent | ElectricalClass::RegulatorOverload => {
                Self::Overcurrent
            }
        }
    }
}
