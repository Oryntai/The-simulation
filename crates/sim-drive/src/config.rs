use std::collections::BTreeMap;
use std::f64::consts::TAU;

use sim_signal::{Rate, Time};

use crate::input::PulseCalibration;

use crate::catalog::{
    BackdriveMode, BrakeMode, ControlMode, DriveEntry, DriveKind, InertiaMode, Parameter as P,
    Quantity, Source, StopMode,
};

#[derive(Clone, Copy)]
pub(crate) struct Electrical {
    pub supply: f64,
    pub limit: f64,
    pub resistance: f64,
    pub inductance: f64,
    pub k: f64,
}

#[derive(Clone, Copy)]
pub(crate) struct Friction {
    pub coulomb: f64,
    pub viscous: f64,
    pub stiction: f64,
    pub stribeck: f64,
    pub band: f64,
}

#[derive(Clone, Copy)]
pub(crate) struct Coupling {
    pub inertia_motor: f64,
    pub half_gap: f64,
    pub stiffness: f64,
    pub damping: f64,
}

#[derive(Clone, Copy)]
pub(crate) struct Stop {
    pub min: f64,
    pub max: f64,
    pub stiffness: f64,
    pub damping: f64,
}

#[derive(Clone, Copy)]
pub(crate) struct Hold {
    pub effort: f64,
    pub stiffness: f64,
}

#[derive(Clone, Copy)]
pub(crate) struct Mechanics {
    /// Радианы ротора на радиан или метр выхода. Обратная величина
    /// переводит момент ротора в обобщённое усилие по равенству работ.
    pub ratio: f64,
    pub forward: f64,
    pub backward: f64,
    pub friction: Friction,
    pub coupling: Option<Coupling>,
    pub stop: Option<Stop>,
    pub lock: Option<Hold>,
    pub brake: Option<Hold>,
}

#[derive(Clone, Copy)]
pub(crate) struct Thermal {
    pub ambient: f64,
    pub initial: [f64; 2],
    pub reference: f64,
    pub alpha: f64,
    pub r_wh: f64,
    pub r_ha: f64,
    pub c_w: f64,
    pub c_h: f64,
    pub winding_fraction: f64,
    pub gear_fraction: f64,
    pub maximum: Option<f64>,
}

#[derive(Clone, Copy)]
pub(crate) struct VelocityControl {
    pub limit: f64,
    pub kp: f64,
    pub ki: f64,
}

#[derive(Clone, Copy)]
pub(crate) enum Mode {
    Pwm,
    Current,
    Velocity(VelocityControl),
    Position {
        min: f64,
        max: f64,
        deadband: f64,
        gain: f64,
        velocity: VelocityControl,
    },
    StepRate {
        limit: f64,
        current: f64,
        poles: f64,
        steps_per_period: f64,
    },
}

#[derive(Clone, Copy)]
pub(crate) struct Config {
    pub electrical: Electrical,
    pub mechanical: Mechanics,
    pub thermal: Option<Thermal>,
    pub mode: Mode,
    pub control: Cadence,
    pub delay: Time,
    pub bus_range: Option<[f64; 2]>,
    pub pulse: Option<PulseCalibration>,
}

impl Config {
    pub(crate) fn initial_resistance(self) -> Result<f64, String> {
        self.resistance_after_start(|t| t.initial[0])
    }

    pub(crate) fn minimum_resistance(self) -> Result<f64, String> {
        // Теплообмен и неотрицательные потери не могут охладить обмотку
        // ниже минимума из начальных температур и температуры среды.
        self.resistance_after_start(|t| t.initial[0].min(t.initial[1]).min(t.ambient))
    }

    fn resistance_after_start(
        self,
        temperature: impl FnOnce(Thermal) -> f64,
    ) -> Result<f64, String> {
        let base = self.electrical.resistance;
        let r = self.thermal.map_or(base, |t| {
            base * (1.0 + t.alpha * (temperature(t) - t.reference))
        });
        if !r.is_finite() || r <= 0.0 {
            return Err(
                "R(T) при начальной температуре должна быть конечной и положительной".into(),
            );
        }
        Ok(r)
    }
}

/// Номинальный период как целая и дробная части наносекунды. Вычисление
/// из точного двоичного представления частоты не накапливает округления.
#[derive(Clone, Copy)]
pub(crate) struct Cadence {
    whole_ns: u64,
    remainder: u128,
    divisor: u128,
}

impl Cadence {
    pub(crate) fn new(hz: f64) -> Option<Self> {
        // Меньшая положительная частота не даёт даже одного ненулевого
        // такта на всей шкале времени. Самая высокая — один такт на нс.
        if !(1.0 / Time(u64::MAX).as_secs()..=1_000_000_000.0).contains(&hz) {
            return None;
        }
        // Формат IEEE-754 binary64: 52 бита дроби и смещение экспоненты 1023.
        let exponent = ((hz.to_bits() >> 52) as i32) - 1023;
        let mantissa = hz.to_bits() & ((1_u64 << 52) - 1) | (1_u64 << 52);
        let divisor = u128::from(mantissa);
        let scaled = 1_000_000_000_u128 << (52 - exponent);
        Some(Self {
            whole_ns: (scaled / divisor).try_into().ok()?,
            remainder: scaled % divisor,
            divisor,
        })
    }

    /// Индекс последнего такта floor(k·p) ≤ now. Дробь точная, такт 0 существует.
    pub(crate) fn last_due(self, now: Time) -> u64 {
        let period = u128::from(self.whole_ns) * self.divisor + self.remainder;
        (((u128::from(now.0) + 1) * self.divisor - 1) / period) as u64
    }

    pub(crate) fn tick(self, index: u64) -> Option<Time> {
        let index = u128::from(index);
        let whole = index * u128::from(self.whole_ns);
        let fraction = index * self.remainder / self.divisor;
        u64::try_from(whole.checked_add(fraction)?).ok().map(Time)
    }
}

struct Resolve<'a> {
    entry: &'a DriveEntry,
    overrides: BTreeMap<P, f64>,
    notes: Vec<String>,
}

impl Resolve<'_> {
    fn value(&self, key: P) -> Option<f64> {
        self.overrides
            .get(&key)
            .copied()
            .or_else(|| self.entry.parameters.get(&key).and_then(Quantity::value))
    }

    fn missing(&self, key: P) -> String {
        let method = match self.entry.parameters.get(&key) {
            Some(Quantity::Unknown(data)) => &data.bench,
            _ => key.bench(),
        };
        format!("{} неизвестен. Как снять на стенде: {method}", key.name())
    }

    fn get(&self, key: P) -> Result<f64, String> {
        self.value(key).ok_or_else(|| self.missing(key))
    }

    fn check(&self, key: P, valid: impl FnOnce(f64) -> bool, message: &str) -> Result<f64, String> {
        let value = self.get(key)?;
        if !valid(value) {
            return Err(format!("{}: {message}", key.name()));
        }
        Ok(value)
    }

    fn positive(&self, key: P) -> Result<f64, String> {
        self.check(key, |v| v > 0.0, "требуется положительное число")
    }

    fn nonnegative(&self, key: P) -> Result<f64, String> {
        self.check(key, |v| v >= 0.0, "требуется неотрицательное число")
    }

    fn fraction(&self, key: P, positive: bool) -> Result<f64, String> {
        self.check(
            key,
            |v| v <= 1.0 && (v > 0.0 || (!positive && v == 0.0)),
            "доля вне допустимого диапазона до 1",
        )
    }

    fn temperature(&self, key: P) -> Result<f64, String> {
        // Точное смещение шкал Кельвина и Цельсия по определению СИ.
        self.check(key, |v| v >= -273.15, "температура ниже абсолютного нуля")
    }

    fn velocity(&self) -> Result<VelocityControl, String> {
        let kp = self.nonnegative(P::SpeedKp)?;
        let ki = self.nonnegative(P::SpeedKi)?;
        if kp == 0.0 && ki == 0.0 {
            return Err("speed_kp и speed_ki одновременно равны нулю".into());
        }
        Ok(VelocityControl {
            limit: self.positive(P::SpeedLimit)?,
            kp,
            ki,
        })
    }

    fn thermal(&mut self) -> Result<Option<Thermal>, String> {
        let required = [
            P::ThermalWh,
            P::ThermalHa,
            P::TauWinding,
            P::TauHousing,
            P::Ambient,
            P::ResistanceRef,
            P::ResistanceAlpha,
            P::WindingLossFraction,
            P::GearLossToHousing,
        ];
        let initial_keys = [P::InitialWinding, P::InitialHousing];
        let explicit_initial = initial_keys
            .into_iter()
            .any(|key| self.value(key).is_some());
        let mut complete = true;
        for key in required {
            if self.value(key).is_none() {
                if explicit_initial {
                    return Err(format!(
                        "начальные температуры требуют полную тепловую сеть. {}",
                        self.missing(key)
                    ));
                }
                complete = false;
                self.notes.push(format!(
                    "нагрев и изменение R(T) не рассчитаны. {}",
                    self.missing(key)
                ));
            }
        }
        if !complete {
            return Ok(None);
        }
        let r_wh = self.positive(P::ThermalWh)?;
        let r_ha = self.positive(P::ThermalHa)?;
        let c_w = self.positive(P::TauWinding)? / r_wh;
        let c_h = self.positive(P::TauHousing)? / r_ha;
        if !c_w.is_finite() || c_w <= 0.0 || !c_h.is_finite() || c_h <= 0.0 {
            return Err("теплоёмкости τ/R вне численного диапазона".into());
        }
        let maximum = if self.value(P::MaxWinding).is_some() {
            Some(self.temperature(P::MaxWinding)?)
        } else {
            self.notes.push(format!(
                "перегрев не оценён. {}",
                self.missing(P::MaxWinding)
            ));
            None
        };
        let ambient = self.temperature(P::Ambient)?;
        let mut initial = [ambient; 2];
        for (key, value) in initial_keys.into_iter().zip(&mut initial) {
            if self.value(key).is_some() {
                *value = self.temperature(key)?;
            }
        }
        Ok(Some(Thermal {
            ambient,
            initial,
            reference: self.temperature(P::ResistanceRef)?,
            alpha: self.nonnegative(P::ResistanceAlpha)?,
            r_wh,
            r_ha,
            c_w,
            c_h,
            winding_fraction: self.fraction(P::WindingLossFraction, false)?,
            gear_fraction: self.fraction(P::GearLossToHousing, false)?,
            maximum,
        }))
    }
    fn pulse(&self, mode: Mode, delay: Time) -> Result<PulseCalibration, String> {
        let Mode::Position { min, max, .. } = mode else {
            return Err("импульсный вход требует режим position".into());
        };
        let period = [
            self.positive(P::PulsePeriodMin)?,
            self.positive(P::PulsePeriodMax)?,
        ];
        let width = [
            self.positive(P::PulseWidthMin)?,
            self.positive(P::PulseWidthMax)?,
        ];
        if period[0] > period[1] || width[0] >= width[1] || width[1] > period[0] {
            return Err(
                "калибровка требует period_min ≤ period_max и width_min < width_max ≤ period_min"
                    .into(),
            );
        }
        let position = [
            self.get(P::PulsePositionAtMinWidth)?,
            self.get(P::PulsePositionAtMaxWidth)?,
        ];
        if !position.into_iter().all(|p| (min..=max).contains(&p))
            || position[0] == position[1]
            || !(position[1] - position[0]).is_finite()
        {
            return Err(
                "калибровка ширины требует две разные координаты внутри пределов регулятора".into(),
            );
        }
        let seconds = self.positive(P::PulseTimeout)?;
        let timeout = Time::from_secs(seconds).ok_or("pulse_timeout_s вне шкалы времени")?;
        if timeout.as_secs() < period[1] || timeout <= delay {
            return Err(
                "pulse_timeout_s должен покрывать максимальный период и превышать задержку команды"
                    .into(),
            );
        }
        Ok(PulseCalibration {
            period,
            width,
            position,
            timeout,
        })
    }
}

#[derive(Clone, Copy)]
pub(crate) struct Features {
    pub bus: bool,
    pub bridge: bool,
    pub pulse: bool,
}

pub(crate) fn resolve(
    entry: &DriveEntry,
    params: &BTreeMap<String, f64>,
    rate: Rate,
    armature: f64,
    features: Features,
) -> Result<(Config, Vec<String>), String> {
    let mut r = Resolve {
        entry,
        overrides: BTreeMap::new(),
        notes: Vec::new(),
    };
    for (key, &value) in params {
        let parameter = P::parse(key).ok_or_else(|| format!("неизвестный параметр «{key}»"))?;
        parameter
            .validate(value)
            .map_err(|err| format!("{key}: {err}"))?;
        r.overrides.insert(parameter, value);
        r.notes.push(format!(
            "{key}: значение экземпляра в params, происхождение в CompiledDrive не указано"
        ));
    }
    for (key, quantity) in entry.quantities() {
        if let Quantity::Known(s) = quantity
            && s.source == Source::Estimate
            && !P::parse(key).is_some_and(|p| r.overrides.contains_key(&p))
        {
            r.notes.push(format!("{key}: оценка, {}", s.reference));
        }
    }
    if !armature.is_finite() || armature < 0.0 {
        return Err("armature должна быть конечной и неотрицательной".into());
    }
    if entry.inertia == InertiaMode::Internal && armature > 0.0 {
        return Err("armature уже задана в MJCF: двойной учёт инерции ротора".into());
    }

    r.notes.push("неявный шаг проверяет локальный отклик оси; пики между тактами, задержка внешних команд и сходимость контактов по сетке требуют отдельной сверки".into());
    if features.bus {
        r.notes.push("напряжение шины задаёт внешний источник; его импеданс, реакция на рекуперацию и защита шины требуют отдельной модели".into());
    } else {
        r.notes.push("напряжение шины задано постоянным; просадка питания, подъём напряжения от рекуперации и защита шины не рассчитаны".into());
    }
    let electrical = Electrical {
        supply: r.positive(P::Supply)?,
        limit: r.positive(P::LimitCurrent)?,
        resistance: r.positive(P::Resistance)?,
        inductance: r.nonnegative(P::Inductance)?,
        k: r.positive(P::MotorConstant)?,
    };
    if r.value(P::VoltageMax).is_some() && electrical.supply > r.positive(P::VoltageMax)? {
        return Err("supply_v выше voltage_max_v".into());
    }
    if r.value(P::VoltageMin).is_some() && electrical.supply < r.nonnegative(P::VoltageMin)? {
        return Err("supply_v ниже voltage_min_v".into());
    }
    for key in [P::VoltageMin, P::VoltageMax] {
        if r.value(key).is_none() {
            r.notes.push(format!(
                "допустимость напряжения питания оценена не полностью. {}",
                r.missing(key)
            ));
        }
    }
    let bus_range = features
        .bus
        .then(|| Ok::<_, String>([r.nonnegative(P::VoltageMin)?, r.positive(P::VoltageMax)?]))
        .transpose()?;
    if electrical.inductance == 0.0 {
        r.notes
            .push("нулевая индуктивность: переходный процесс тока не рассчитан".into());
    }
    let ratio = r.positive(P::Ratio)?
        / if entry.kind == DriveKind::Linear {
            r.positive(P::LinearTravel)? / TAU
        } else {
            1.0 // Радиан выхода вращательного привода уже выражен в СИ.
        };
    if !ratio.is_finite() || ratio <= 0.0 {
        return Err("коэффициент передачи вне численного диапазона".into());
    }
    let coupling = match entry.inertia {
        InertiaMode::Internal => {
            let inertia_motor = r.positive(P::RotorInertia)? + r.nonnegative(P::GearInertia)?;
            let stiffness = r.positive(P::Stiffness)?;
            let damping = r.nonnegative(P::Damping)?;
            let half_gap = r.nonnegative(P::Backlash)? / 2.0; // В каталоге полный зазор.
            if !inertia_motor.is_finite() {
                return Err("сумма инерций ротора и входа передачи переполнена".into());
            }
            Some(Coupling {
                inertia_motor,
                half_gap,
                stiffness,
                damping,
            })
        }
        InertiaMode::Mjcf => {
            if armature == 0.0 {
                return Err("inertia = mjcf требует положительную armature на оси привода".into());
            }
            if r.nonnegative(P::Backlash)? != 0.0 {
                return Err("люфт требует inertia = internal и податливую передачу".into());
            }
            if r.value(P::Stiffness).is_some() || r.value(P::Damping).is_some() {
                return Err(
                    "coupling_stiffness и coupling_damping требуют inertia = internal".into(),
                );
            }
            match (r.value(P::RotorInertia), r.value(P::GearInertia)) {
                (Some(rotor), Some(gear)) => {
                    let expected = (rotor + gear) * ratio * ratio;
                    // Допуск округления для сложения, умножений и чтения MJCF.
                    let tolerance = 16.0 * f64::EPSILON * expected.abs().max(armature);
                    if rotor < 0.0 || gear < 0.0 || !expected.is_finite() || (expected - armature).abs() > tolerance {
                        return Err(format!("armature {armature} расходится с приведённой инерцией (J_rotor + J_gear)·N² = {expected}"));
                    }
                }
                _ => r.notes.push("приведённая инерция берётся из armature и не сверена с ротором и входом передачи".into()),
            }
            None
        }
    };
    let stop = match entry.stop {
        StopMode::None => None,
        StopMode::Unknown => {
            r.notes.push("механический упор не рассчитан. Измерить координаты, жёсткость и затухание контакта".into());
            None
        }
        StopMode::Compliant => {
            let min = r.get(P::StopMin)?;
            let max = r.get(P::StopMax)?;
            if min >= max {
                return Err("stop_min должен быть меньше stop_max".into());
            }
            Some(Stop {
                min,
                max,
                stiffness: r.positive(P::StopStiffness)?,
                damping: r.nonnegative(P::StopDamping)?,
            })
        }
    };
    let lock = match entry.backdrive {
        BackdriveMode::Reversible => None,
        BackdriveMode::Unknown => {
            r.notes.push("обратимость неизвестна, удержание передачей отключено. Приложить обратную нагрузку без питания".into());
            None
        }
        BackdriveMode::SelfLocking => {
            r.notes.push("самоторможение рассчитано при отсутствии питания двигателя; проворот под напряжением требует стендовой сверки".into());
            Some(Hold {
                effort: r.positive(P::HoldEffort)?,
                stiffness: r.positive(P::HoldStiffness)?,
            })
        }
    };
    let brake = match entry.brake {
        BrakeMode::None => None,
        BrakeMode::Unknown => {
            r.notes.push("тормоз не рассчитан. Проверить наличие, условие срабатывания и усилие удержания при снятии питания".into());
            None
        }
        BrakeMode::Holding => {
            if !entry.power_input {
                r.notes.push("тормоз включён до первой принятой команды; для снятия питания в прогоне требуется power_input".into());
            }
            Some(Hold {
                effort: r.positive(P::BrakeEffort)?,
                stiffness: r.positive(P::BrakeStiffness)?,
            })
        }
    };
    let friction = Friction {
        coulomb: r.nonnegative(P::Coulomb)?,
        viscous: r.nonnegative(P::Viscous)?,
        stiction: r.nonnegative(P::StaticFriction)?,
        stribeck: r.positive(P::StribeckSpeed)?,
        band: r.positive(P::FrictionBand)?,
    };
    if friction.stiction < friction.coulomb {
        return Err("static_friction_nm меньше coulomb_nm".into());
    }
    if friction.stiction > 0.0 {
        r.notes.push("трение регуляризовано около нулевой скорости; удержание одним статическим трением не доказано".into());
    }
    let mechanical = Mechanics {
        ratio,
        forward: r.fraction(P::ForwardEfficiency, true)?,
        backward: r.fraction(P::BackwardEfficiency, true)?,
        friction,
        coupling,
        stop,
        lock,
        brake,
    };
    if mechanical.forward < 1.0 || mechanical.backward < 1.0 {
        r.notes.push("КПД меняет ветвь в нуле потока мощности; трогание под обратной нагрузкой и удержание около нулевой скорости требуют стендовой сверки".into());
    }
    let control_hz = r.check(
        P::ControlHz,
        |v| v > 0.0 && (entry.mode == ControlMode::Pwm || v <= rate.as_hz()),
        "требуется положительная частота; для регулятора с обратной связью — не выше частоты физики",
    )?;
    let control = Cadence::new(control_hz).ok_or("control_hz вне шкалы времени")?;
    let delay = Time::from_secs(r.get(P::BusDelay)?).ok_or("bus_delay_s вне шкалы времени")?;

    if (entry.mode == ControlMode::StepRate) != (entry.kind == DriveKind::Stepper) {
        return Err("шаговый привод требует режим step_rate, для остальных он неприменим".into());
    }
    let mode = match entry.mode {
        ControlMode::Pwm => Mode::Pwm,
        ControlMode::Current => Mode::Current,
        ControlMode::Velocity => Mode::Velocity(r.velocity()?),
        ControlMode::Position => {
            let min = r.get(P::PositionMin)?;
            let max = r.get(P::PositionMax)?;
            if min >= max {
                return Err("position_min должен быть меньше position_max".into());
            }
            Mode::Position {
                min,
                max,
                deadband: r.nonnegative(P::Deadband)?,
                gain: r.positive(P::PositionGain)?,
                velocity: r.velocity()?,
            }
        }
        ControlMode::StepRate => {
            // Двухфазная коммутация: четыре полношаговых состояния за электрический оборот.
            let integer = |v: f64| v >= 1.0 && v.fract() == 0.0 && v <= f64::from(u32::MAX);
            let steps_per_period =
                4.0 * r.check(P::Microsteps, integer, "требуется положительное целое u32")?;
            let poles = r.check(P::PolePairs, integer, "требуется положительное целое u32")?;
            let current = r.positive(P::StepCurrent)?;
            if current > electrical.limit {
                return Err("step_current_a выше current_limit_a".into());
            }
            let limit = r.positive(P::MaxStepRate)?;
            if limit > control_hz {
                return Err(
                    "max_step_rate выше control_hz: регулятор пропустит фазовые состояния".into(),
                );
            }
            Mode::StepRate {
                limit,
                current,
                poles,
                steps_per_period,
            }
        }
    };
    if entry.kind == DriveKind::Bldc {
        r.notes.push("усреднённая модель BLDC: пульсации фазных токов, коммутация и ослабление поля не рассчитаны".into());
    }
    if entry.kind == DriveKind::Stepper {
        r.notes.push("срыв шагов считается по проскокам электрического периода; фактическая граница pull-out требует измерения для драйвера и нагрузки".into());
    }
    if entry.builtin_driver.is_some() {
        r.notes.push("встроенный драйвер представлен общим регулятором; прошивка и фирменные защиты не исполняются".into());
    }
    if !matches!(mode, Mode::Pwm) {
        r.notes.push("коммутация и регулятор дискретны: пики тока и запасы замкнутого контура проверяются отдельно на частотах контроллера, шины и физики".into());
    }
    if features.bridge {
        r.notes.push("мост идеализирован: диоды без падения напряжения, замыкание без сопротивления ключей; коммутация и защита при сверхтоке требуют стендовой сверки".into());
    } else if entry.power_input || features.pulse {
        r.notes.push("отключение моста идеализировано: энергия обмотки рассеивается вне двигателя, схема гашения и пульсации питания не рассчитаны".into());
    }
    if coupling.is_some() {
        r.notes.push("связь ротора с миром обновляется на тактах физики; сходимость длительного движения и потерь при смене направления требует сгущения сетки".into());
    }
    if stop.is_some()
        || lock.is_some()
        || brake.is_some()
        || coupling.is_some_and(|c| c.half_gap > 0.0)
    {
        r.notes.push("срабатывание механических пределов проверяется на сетке физики; моменты переключения, пики усилия и тепловые потери удержания требуют отдельной сверки".into());
    }
    let config = Config {
        electrical,
        mechanical,
        thermal: r.thermal()?,
        mode,
        control,
        delay,
        bus_range,
        pulse: features.pulse.then(|| r.pulse(mode, delay)).transpose()?,
    };
    config.initial_resistance()?;
    config.minimum_resistance()?;
    Ok((config, r.notes))
}
