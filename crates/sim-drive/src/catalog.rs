use std::collections::BTreeMap;
use std::path::Path;

use serde::Deserialize;

/// Число СИ и его происхождение. Поля двух форм взаимоисключающие.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(untagged)]
pub enum Quantity {
    Known(Sourced),
    Unknown(Unmeasured),
}

#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Sourced {
    pub value: f64,
    pub source: Source,
    pub reference: String,
}

#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Unmeasured {
    pub unknown: bool,
    pub bench: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Source {
    Datasheet,
    Paper,
    Measured,
    Derived,
    Estimate,
}

impl Quantity {
    pub fn value(&self) -> Option<f64> {
        match self {
            Self::Known(number) => Some(number.value),
            Self::Unknown(_) => None,
        }
    }

    fn validate(&self) -> Result<(), &'static str> {
        match self {
            Self::Known(s) if s.value.is_finite() && !s.reference.trim().is_empty() => Ok(()),
            Self::Known(_) => Err("требуется конечное число и непустая ссылка на источник"),
            Self::Unknown(u) if u.unknown && !u.bench.trim().is_empty() => Ok(()),
            Self::Unknown(_) => Err("требуется unknown = true и непустой метод замера bench"),
        }
    }
}

macro_rules! options {
    ($name:ident { $($variant:ident),+ }) => {
        #[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize)]
        #[serde(rename_all = "snake_case")]
        pub enum $name { $($variant),+ }
    };
}

options!(DriveKind {
    Dc,
    Bldc,
    Stepper,
    HobbyServo,
    BusServo,
    Linear
});
options!(ControlMode {
    Pwm,
    Current,
    Velocity,
    Position,
    StepRate
});
options!(InertiaMode { Internal, Mjcf });
options!(BackdriveMode {
    Reversible,
    SelfLocking,
    Unknown
});
options!(BrakeMode {
    None,
    Holding,
    Unknown
});
options!(StopMode {
    None,
    Compliant,
    Unknown
});
options!(Magnet {
    NdFeB,
    Ferrite,
    Unknown
});
options!(LegacyClass {
    GearedBldc,
    Qdd,
    SmartServo,
    HobbyServo,
    BrushedGearmotor,
    Stepper
});

macro_rules! parameters {
    ($($variant:ident => $key:literal, $bench:literal;)+) => {
        #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Deserialize)]
        pub enum Parameter {
            $(#[serde(rename = $key)] $variant,)+
        }

        impl Parameter {
            pub(crate) fn parse(key: &str) -> Option<Self> {
                match key {
                    $($key => Some(Self::$variant),)+
                    _ => None,
                }
            }

            pub fn name(self) -> &'static str {
                match self { $(Self::$variant => $key,)+ }
            }

            pub(crate) fn bench(self) -> &'static str {
                match self { $(Self::$variant => $bench,)+ }
            }
        }
    };
}

parameters! {
    VoltageNominal => "voltage_nominal_v", "проверить номинал на маркировке и измерить шину под нагрузкой";
    VoltageMin => "voltage_min_v", "понижать напряжение шины под нагрузкой до порога отключения";
    VoltageMax => "voltage_max_v", "определить допустимое напряжение шины по паспорту и испытанию драйвера";
    RatedTorque => "rated_torque_nm", "выдержать длительную нагрузку на динамометре до теплового равновесия";
    PeakTorque => "peak_torque_nm", "снять кратковременный предел на динамометре вместе с длительностью";
    NoLoadSpeed => "no_load_speed_rpm", "измерить скорость выхода без нагрузки при номинальном напряжении";
    RatedSpeed => "rated_speed_rpm", "измерить скорость выхода под номинальной нагрузкой";
    NoLoadCurrent => "no_load_current_a", "измерить ток при номинальном напряжении без нагрузки";
    StallCurrent => "stall_current_a", "измерить кратковременный ток при блокировке выхода и данном напряжении";
    RatedCurrent => "rated_current_a", "снять длительный ток при допустимом установившемся нагреве";
    PeakCurrent => "peak_current_a", "снять пиковый ток вместе с длительностью импульса";
    StandbyCurrent => "standby_current_a", "измерить ток питания при покоящемся валу";
    Resistance => "resistance_ohm", "измерить сопротивление обмотки четырёхпроводным методом при известной температуре";
    Inductance => "inductance_h", "снять переход тока коротким импульсом на неподвижном валу или измерить LCR-метром";
    Kt => "kt_motor_nm_per_a", "снять наклон момента по току на валу двигателя";
    Ke => "ke_motor_v_s", "раскрутить разомкнутый двигатель и измерить ЭДС и скорость";
    Kv => "kv_rpm_per_v", "снять отношение скорости вала к противо-ЭДС";
    Ratio => "gear_ratio", "сопоставить перемещение вала двигателя и выходного вала";
    ForwardEfficiency => "gear_efficiency", "измерить механическую мощность на входе и выходе при прямом потоке";
    RotorInertia => "rotor_inertia_kg_m2", "снять ускорение ненагруженного ротора от известного момента";
    GearInertia => "gear_input_inertia_kg_m2", "снять ускорение входа передачи от известного момента";
    Mass => "mass_kg", "взвесить привод";
    ThermalWh => "r_th_winding_housing_k_per_w", "оценить по перепаду обмотка–корпус при известной тепловой мощности";
    ThermalHa => "r_th_housing_ambient_k_per_w", "оценить по перепаду корпус–среда при известной тепловой мощности";
    TauWinding => "tau_winding_s", "фитировать два узла по переходу температуры обмотки при ступени мощности";
    TauHousing => "tau_housing_s", "фитировать два узла по переходу температуры корпуса при ступени мощности";
    ThermalTotal => "r_th_total_k_per_w", "измерить установившийся перепад обмотка–среда при известной мощности";
    MaxWinding => "max_winding_temp_c", "уточнить предел по классу изоляции и порогу тепловой защиты";
    MinAmbient => "min_ambient_c", "измерить нижнюю границу рабочего диапазона в термокамере";
    MaxAmbient => "max_ambient_c", "измерить верхнюю границу рабочего диапазона в термокамере";
    HeatedPhases => "heated_phases", "записать ток всех обмоток за цикл коммутации";
    PolePairs => "stepper_pole_pairs", "посчитать периоды противо-ЭДС за оборот";
    Supply => "supply_v", "записать напряжение шины на рабочем цикле";
    LimitCurrent => "current_limit_a", "снять уставку ограничителя тока драйвера по осциллограмме";
    MotorConstant => "motor_constant", "снять момент на ток и ЭДС на скорость в одной энергетической системе единиц";
    ResistanceRef => "resistance_ref_c", "записать температуру обмотки при замере сопротивления";
    ResistanceAlpha => "resistance_alpha_per_k", "снять сопротивление обмотки при двух известных температурах";
    InitialWinding => "initial_winding_temp_c", "измерить температуру обмотки по сопротивлению перед пуском";
    InitialHousing => "initial_housing_temp_c", "измерить температуру корпуса перед пуском контактным датчиком";
    PulsePeriodMin => "pulse_period_min_s", "уменьшать период при постоянной ширине и фиксировать границу приёма";
    PulsePeriodMax => "pulse_period_max_s", "увеличивать период при постоянной ширине и фиксировать границу приёма";
    PulseWidthMin => "pulse_width_min_s", "уменьшать ширину при допустимом периоде и снять границу диапазона команды";
    PulseWidthMax => "pulse_width_max_s", "увеличивать ширину при допустимом периоде и снять границу диапазона команды";
    PulsePositionAtMinWidth => "pulse_position_at_min_width", "измерить координату выхода при минимальной допустимой ширине";
    PulsePositionAtMaxWidth => "pulse_position_at_max_width", "измерить координату выхода при максимальной допустимой ширине";
    PulseTimeout => "pulse_timeout_s", "прервать импульсы под контролируемой нагрузкой и измерить время до снятия команды";
    Ambient => "ambient_c", "измерить температуру среды у корпуса";
    WindingLossFraction => "winding_loss_fraction", "идентифицировать долю тепла двигателя в обмотке по нагреву холостого хода";
    GearLossToHousing => "gear_loss_to_housing_fraction", "измерить тепловой поток от передачи к корпусу привода";
    BackwardEfficiency => "backward_efficiency", "нагрузить передачу с выхода и замерить мощности на обеих сторонах";
    LinearTravel => "linear_travel_m_per_rev", "измерить ход за оборот выходного вала передачи";
    Coulomb => "coulomb_nm", "снять постоянную составляющую момента трения после страгивания";
    Viscous => "viscous_nm_per_rad_s", "снять наклон момента трения по скорости";
    StaticFriction => "static_friction_nm", "медленно повышать момент на валу до страгивания";
    StribeckSpeed => "stribeck_rad_s", "снять кривую трения на малых скоростях после страгивания";
    FrictionBand => "friction_band_rad_s", "снять разрешение скоростного стенда и проверить сходимость полосы регуляризации";
    Backlash => "backlash", "измерить полный свободный ход выхода между двумя нагруженными зубьями";
    Stiffness => "coupling_stiffness", "замерить наклон сила–перемещение или момент–угол при зацеплении";
    Damping => "coupling_damping", "снять затухание от ступенчатого смещения передачи";
    HoldEffort => "hold_effort", "подать обратную нагрузку без питания и измерить усилие начала проворота";
    HoldStiffness => "hold_stiffness", "измерить жёсткость удержания медленным нагружением";
    BrakeEffort => "brake_effort", "снять усилие начала скольжения при включённом тормозе";
    BrakeStiffness => "brake_stiffness", "снять жёсткость заторможенного выхода";
    StopMin => "stop_min", "медленно подвезти выход к нижнему механическому упору и измерить координату";
    StopMax => "stop_max", "медленно подвезти выход к верхнему механическому упору и измерить координату";
    StopStiffness => "stop_stiffness", "измерить жёсткость контакта упора";
    StopDamping => "stop_damping", "идентифицировать затухание при контролируемом ударе об упор";
    ControlHz => "control_hz", "измерить период внутренних обновлений по реакции на серию меток";
    BusDelay => "bus_delay_s", "синхронно записать момент передачи и момент принятия команды";
    PositionMin => "position_min", "проверить нижнюю разрешённую регулятором координату";
    PositionMax => "position_max", "проверить верхнюю разрешённую регулятором координату";
    SpeedLimit => "speed_limit", "измерить предел задания выходной скорости";
    PositionGain => "position_gain", "снять наклон задания скорости по ошибке положения";
    Deadband => "position_deadband", "медленно менять команду при неподвижном выходе и найти порог реакции";
    SpeedKp => "speed_kp", "снять мгновенный наклон задания тока по ошибке скорости";
    SpeedKi => "speed_ki", "снять скорость нарастания задания тока при постоянной ошибке скорости";
    Microsteps => "microsteps", "проверить настройку драйвера: микрошагов на полный шаг";
    MaxStepRate => "max_step_rate", "измерить действующий предел частоты шаговых команд";
    StepCurrent => "step_current_a", "измерить амплитуду тока фазы в режиме микрошага";
}

impl Parameter {
    pub(crate) fn validate(self, value: f64) -> Result<(), &'static str> {
        if !value.is_finite() {
            return Err("требуется конечное число");
        }
        match self {
            Self::PositionMin
            | Self::PositionMax
            | Self::StopMin
            | Self::StopMax
            | Self::PulsePositionAtMinWidth
            | Self::PulsePositionAtMaxWidth => Ok(()),
            // Точное смещение шкал Кельвина и Цельсия по определению СИ.
            Self::Ambient
            | Self::InitialWinding
            | Self::InitialHousing
            | Self::ResistanceRef
            | Self::MaxWinding
            | Self::MinAmbient
            | Self::MaxAmbient
                if value >= -273.15 =>
            {
                Ok(())
            }
            Self::Ambient
            | Self::InitialWinding
            | Self::InitialHousing
            | Self::ResistanceRef
            | Self::MaxWinding
            | Self::MinAmbient
            | Self::MaxAmbient => Err("температура ниже абсолютного нуля"),
            Self::ForwardEfficiency | Self::BackwardEfficiency
                if !(0.0 < value && value <= 1.0) =>
            {
                Err("КПД должен быть больше нуля и не больше 1")
            }
            Self::WindingLossFraction | Self::GearLossToHousing
                if !(0.0..=1.0).contains(&value) =>
            {
                Err("доля должна принадлежать отрезку [0, 1]")
            }
            _ if value < 0.0 => Err("требуется неотрицательное число"),
            _ => Ok(()),
        }
    }
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VoltagePoint {
    pub voltage_v: Quantity,
    pub stall_torque_nm: Quantity,
    pub no_load_rpm: Quantity,
    pub stall_current_a: Quantity,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DriveEntry {
    pub id: String,
    pub name: String,
    pub kind: DriveKind,
    pub mode: ControlMode,
    pub inertia: InertiaMode,
    pub backdrive: BackdriveMode,
    pub brake: BrakeMode,
    pub stop: StopMode,
    /// Дополнительный вход <имя входа усилия>-power принимает 0 или 1.
    #[serde(default)]
    pub power_input: bool,
    #[serde(default)]
    pub what_is: String,
    pub legacy_class: Option<LegacyClass>,
    pub magnet: Option<Magnet>,
    pub builtin_driver: Option<String>,
    pub backdrive_source: Option<String>,
    pub backdrive_torque_nm: Option<Quantity>,
    pub brake_torque_nm: Option<Quantity>,
    #[serde(default)]
    pub sources: Vec<String>,
    #[serde(default)]
    pub caveats: Vec<String>,
    #[serde(default)]
    pub voltage_points: Vec<VoltagePoint>,
    pub parameters: BTreeMap<Parameter, Quantity>,
}

impl DriveEntry {
    fn validate(&self) -> Result<(), String> {
        if self.name.trim().is_empty() {
            return Err("пустое имя позиции".into());
        }
        if self
            .backdrive_source
            .as_ref()
            .is_some_and(|s| s.trim().is_empty())
        {
            return Err("пустой источник backdrive_source".into());
        }
        for (key, quantity) in &self.parameters {
            quantity
                .validate()
                .map_err(|err| format!("{}: {err}", key.name()))?;
            if let Some(value) = quantity.value() {
                key.validate(value)
                    .map_err(|err| format!("{}: {err}", key.name()))?;
            }
        }
        for (key, quantity) in [
            ("backdrive_torque_nm", &self.backdrive_torque_nm),
            ("brake_torque_nm", &self.brake_torque_nm),
        ] {
            if let Some(quantity) = quantity {
                quantity.validate().map_err(|err| format!("{key}: {err}"))?;
            }
        }
        let mut previous = 0.0;
        for (index, point) in self.voltage_points.iter().enumerate() {
            for (key, quantity) in [
                ("voltage_v", &point.voltage_v),
                ("stall_torque_nm", &point.stall_torque_nm),
                ("no_load_rpm", &point.no_load_rpm),
                ("stall_current_a", &point.stall_current_a),
            ] {
                quantity
                    .validate()
                    .map_err(|err| format!("voltage_points[{index}].{key}: {err}"))?;
                if quantity.value().is_some_and(|value| value <= 0.0) {
                    return Err(format!(
                        "voltage_points[{index}].{key}: требуется положительное число"
                    ));
                }
            }
            let value = point.voltage_v.value().ok_or("точка без напряжения")?;
            if value <= previous {
                return Err("напряжения паспортных точек должны строго возрастать".into());
            }
            previous = value;
        }
        Ok(())
    }

    pub(crate) fn quantities(&self) -> impl Iterator<Item = (&str, &Quantity)> {
        self.parameters
            .iter()
            .map(|(key, value)| (key.name(), value))
            .chain(
                self.backdrive_torque_nm
                    .iter()
                    .map(|q| ("backdrive_torque_nm", q)),
            )
            .chain(self.brake_torque_nm.iter().map(|q| ("brake_torque_nm", q)))
            .chain(self.voltage_points.iter().flat_map(|p| {
                [
                    ("voltage_points.voltage_v", &p.voltage_v),
                    ("voltage_points.stall_torque_nm", &p.stall_torque_nm),
                    ("voltage_points.no_load_rpm", &p.no_load_rpm),
                    ("voltage_points.stall_current_a", &p.stall_current_a),
                ]
            }))
    }
}

/// Каталог загружается из переданного каталога файлов, в порядке идентификаторов.
#[derive(Clone, Debug)]
pub struct DriveCatalog {
    entries: BTreeMap<String, DriveEntry>,
}

impl DriveCatalog {
    pub fn load(directory: impl AsRef<Path>) -> Result<Self, String> {
        let directory = directory.as_ref();
        let report = |err: std::io::Error| format!("{}: {err}", directory.display());
        let mut paths = std::fs::read_dir(directory)
            .map_err(report)?
            .map(|entry| entry.map(|item| item.path()))
            .collect::<Result<Vec<_>, _>>()
            .map_err(report)?;
        paths.sort();
        let mut entries = BTreeMap::new();
        for path in paths
            .into_iter()
            .filter(|path| path.extension().is_some_and(|ext| ext == "toml"))
        {
            let text = std::fs::read_to_string(&path)
                .map_err(|err| format!("{}: {err}", path.display()))?;
            let entry: DriveEntry =
                toml::from_str(&text).map_err(|err| format!("{}: {err}", path.display()))?;
            if !valid_id(&entry.id)
                || path.file_stem().and_then(|stem| stem.to_str()) != Some(entry.id.as_str())
            {
                return Err(format!(
                    "{}: требуется имя файла <id>.toml с тем же id из строчных ASCII-букв, цифр и дефисов",
                    path.display()
                ));
            }
            entry
                .validate()
                .map_err(|err| format!("{}: {err}", path.display()))?;
            entries.insert(entry.id.clone(), entry);
        }
        Ok(Self { entries })
    }

    pub fn get(&self, id: &str) -> Option<&DriveEntry> {
        self.entries.get(id)
    }

    pub fn iter(&self) -> impl ExactSizeIterator<Item = &DriveEntry> {
        self.entries.values()
    }
}

fn valid_id(id: &str) -> bool {
    id.split('-').all(|word| {
        !word.is_empty()
            && word
                .bytes()
                .all(|b| b.is_ascii_lowercase() || b.is_ascii_digit())
    })
}
