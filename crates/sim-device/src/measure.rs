//! Замеры на настоящем устройстве против предсказания симуляции.
//!
//! Правило одно: если реальность опаснее расчёта больше допуска, симуляция была слишком
//! оптимистична, и такой билд не проходит, какими бы зелёными ни были остальные проверки.

/// В какую сторону величина опасна.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Danger {
    /// Опасно, когда больше расчёта (сила, температура, масса).
    Higher,
    /// Опасно, когда меньше расчёта (момент привода, ёмкость).
    Lower,
}

/// Сколько расхождения считаем совпадением. Это оценка точности обычных измерений на
/// стенде: весы, динамометр, термопара.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Tolerance {
    Relative(f64),
    Absolute(f64),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Quantity {
    TotalMass,
    LinkDeflection,
    JointRequiredTorque,
    ActuatorPeakTorque,
    JointSpeed,
    ImpactForce,
    ClampForce,
    MotorTemperature,
    BoardTemperature,
    BatteryTemperature,
    SkinSurfaceTemperature,
    BatteryCurrent,
}

impl Quantity {
    pub const ALL: [Quantity; 12] = [
        Quantity::TotalMass,
        Quantity::LinkDeflection,
        Quantity::JointRequiredTorque,
        Quantity::ActuatorPeakTorque,
        Quantity::JointSpeed,
        Quantity::ImpactForce,
        Quantity::ClampForce,
        Quantity::MotorTemperature,
        Quantity::BoardTemperature,
        Quantity::BatteryTemperature,
        Quantity::SkinSurfaceTemperature,
        Quantity::BatteryCurrent,
    ];

    pub fn key(self) -> &'static str {
        match self {
            Quantity::TotalMass => "total_mass",
            Quantity::LinkDeflection => "link_deflection",
            Quantity::JointRequiredTorque => "joint_required_torque",
            Quantity::ActuatorPeakTorque => "actuator_peak_torque",
            Quantity::JointSpeed => "joint_speed",
            Quantity::ImpactForce => "impact_force",
            Quantity::ClampForce => "clamp_force",
            Quantity::MotorTemperature => "motor_temperature",
            Quantity::BoardTemperature => "board_temperature",
            Quantity::BatteryTemperature => "battery_temperature",
            Quantity::SkinSurfaceTemperature => "skin_surface_temperature",
            Quantity::BatteryCurrent => "battery_current",
        }
    }

    pub fn from_key(s: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|q| q.key() == s)
    }

    pub fn title(self) -> &'static str {
        match self {
            Quantity::TotalMass => "масса устройства",
            Quantity::LinkDeflection => "прогиб звена",
            Quantity::JointRequiredTorque => "нужный момент в шарнире",
            Quantity::ActuatorPeakTorque => "реальный пиковый момент привода",
            Quantity::JointSpeed => "скорость шарнира",
            Quantity::ImpactForce => "сила удара",
            Quantity::ClampForce => "сила зажима",
            Quantity::MotorTemperature => "температура мотора",
            Quantity::BoardTemperature => "температура платы",
            Quantity::BatteryTemperature => "температура батареи",
            Quantity::SkinSurfaceTemperature => "температура поверхности у кожи",
            Quantity::BatteryCurrent => "ток батареи",
        }
    }

    pub fn unit(self) -> &'static str {
        match self {
            Quantity::TotalMass => "кг",
            Quantity::LinkDeflection => "мм",
            Quantity::JointRequiredTorque | Quantity::ActuatorPeakTorque => "Н·м",
            Quantity::JointSpeed => "°/с",
            Quantity::ImpactForce | Quantity::ClampForce => "Н",
            Quantity::MotorTemperature
            | Quantity::BoardTemperature
            | Quantity::BatteryTemperature
            | Quantity::SkinSurfaceTemperature => "°C",
            Quantity::BatteryCurrent => "А",
        }
    }

    pub fn danger(self) -> Danger {
        match self {
            Quantity::ActuatorPeakTorque => Danger::Lower,
            _ => Danger::Higher,
        }
    }

    pub fn tolerance(self) -> Tolerance {
        match self {
            Quantity::TotalMass => Tolerance::Relative(0.05),
            Quantity::LinkDeflection | Quantity::ImpactForce => Tolerance::Relative(0.15),
            Quantity::JointRequiredTorque
            | Quantity::ActuatorPeakTorque
            | Quantity::JointSpeed
            | Quantity::ClampForce
            | Quantity::BatteryCurrent => Tolerance::Relative(0.10),
            Quantity::MotorTemperature | Quantity::BoardTemperature => Tolerance::Absolute(5.0),
            Quantity::BatteryTemperature | Quantity::SkinSurfaceTemperature => {
                Tolerance::Absolute(2.0)
            }
        }
    }

    /// Относится к конкретному звену (шарниру), а не ко всему устройству.
    pub fn per_link(self) -> bool {
        matches!(
            self,
            Quantity::LinkDeflection
                | Quantity::JointRequiredTorque
                | Quantity::ActuatorPeakTorque
                | Quantity::JointSpeed
                | Quantity::ImpactForce
                | Quantity::ClampForce
                | Quantity::MotorTemperature
        )
    }
}

/// Во сколько раз замер может отличаться от верхней границы расчёта и всё ещё считаться замером
/// того же режима. Оценка: рабочая точка обычно ниже худшей в 1–3 раза.
const IMPLAUSIBLE_RATIO: f64 = 5.0;

/// Один замер на стенде.
#[derive(Clone, Debug, PartialEq)]
pub struct Measurement {
    pub quantity: Quantity,
    pub link: Option<usize>,
    pub value: f64,
    /// Чем и как мерили.
    pub note: String,
    /// Отпечаток конструкции, на которой снят замер. Другой отпечаток — замер другого устройства.
    pub design: Option<u64>,
}

/// Отпечаток всего, что влияет на расчёт: файл проекта без названия и замеров плюс образ прошивки.
pub fn design_stamp(d: &crate::Device, firmware: &[u8]) -> u64 {
    let mut c = d.clone();
    c.name.clear();
    c.template.clear();
    c.measurements.clear();
    // FNV-1a, 64 бита: стабильна между запусками и версиями Rust, в отличие от DefaultHasher.
    crate::file::to_text(&c)
        .bytes()
        .chain(firmware.iter().copied())
        .fold(0xcbf2_9ce4_8422_2325, |h, b| {
            (h ^ u64::from(b)).wrapping_mul(0x0000_0100_0000_01b3)
        })
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MeasureStatus {
    /// Совпало в пределах точности замера.
    Agrees,
    /// Реальность опаснее расчёта: симуляция была оптимистична.
    RealIsWorse,
    /// Реальность безопаснее расчёта: запас есть, но модель стоит уточнить.
    RealIsBetter,
    /// Для этой величины у симуляции нет числа.
    NoPrediction,
    /// Замер не похож на это устройство: ноль, масса не сходится, расхождение в разы.
    Suspicious,
}

#[derive(Clone, Debug, PartialEq)]
pub struct MeasureResult {
    pub measurement: Measurement,
    pub predicted: Option<f64>,
    pub status: MeasureStatus,
    pub detail: String,
}

/// Сравнить замер с предсказанием.
pub fn compare(m: &Measurement, predicted: Option<f64>) -> MeasureResult {
    let q = m.quantity;
    let unit = q.unit();
    let done = |status, detail: String| MeasureResult {
        measurement: m.clone(),
        predicted,
        status,
        detail,
    };
    if !m.value.is_finite() {
        return done(
            MeasureStatus::RealIsWorse,
            "замер не число: считать его безопасным нельзя".into(),
        );
    }
    if predicted.is_some_and(|x| !x.is_finite()) {
        return done(
            MeasureStatus::RealIsWorse,
            format!("{}: результат расчёта неконечен", q.title()),
        );
    }
    let Some(p) = predicted else {
        return done(
            MeasureStatus::NoPrediction,
            format!("{}: у симуляции нет числа для сравнения", q.title()),
        );
    };
    let allowed = match q.tolerance() {
        Tolerance::Relative(r) => r * p.abs(),
        Tolerance::Absolute(a) => a,
    };
    let excess = match q.danger() {
        Danger::Higher => m.value - p,
        Danger::Lower => p - m.value,
    };
    let temperature = unit == "°C";
    // Расчёт даёт верхнюю границу, поэтому замер ниже него нормален. Но ноль, масса не по
    // ведомости или расхождение в разы означают другой режим или другое устройство.
    let suspicious = excess <= allowed
        && ((!temperature && m.value <= 0.0)
            || (q == Quantity::TotalMass && excess < -allowed)
            || (!temperature
                && p > 0.0
                && (m.value < p / IMPLAUSIBLE_RATIO || m.value > p * IMPLAUSIBLE_RATIO)));
    if suspicious {
        return done(
            MeasureStatus::Suspicious,
            format!(
                "{}: замер {:.2} {unit}, расчёт {p:.2} {unit}. Расхождение слишком большое, чтобы считать замер подтверждением: проверьте режим, прибор и версию устройства",
                q.title(),
                m.value
            ),
        );
    }
    if !excess.is_finite() || !allowed.is_finite() || excess > allowed {
        done(
            MeasureStatus::RealIsWorse,
            format!(
                "{}: замер {:.2} {unit}, расчёт {p:.2} {unit}. Реальность опаснее расчёта, симуляция была оптимистична",
                q.title(),
                m.value
            ),
        )
    } else if excess < -allowed {
        done(
            MeasureStatus::RealIsBetter,
            format!(
                "{}: замер {:.2} {unit}, расчёт {p:.2} {unit}. Реальность безопаснее, модель можно уточнить",
                q.title(),
                m.value
            ),
        )
    } else {
        done(
            MeasureStatus::Agrees,
            format!(
                "{}: замер {:.2} {unit}, расчёт {p:.2} {unit}, совпало в пределах точности",
                q.title(),
                m.value
            ),
        )
    }
}
