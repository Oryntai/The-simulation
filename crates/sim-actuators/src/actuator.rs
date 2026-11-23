//! Привод: паспортные данные и рабочая прямая «момент–скорость» при заданном напряжении.
//!
//! Модель на выходном валу (после редуктора):
//! V = R(T)·i + K_e·ω, τ = K_t(T)·(i − I₀), где K_e = N·k, K_t = N·η·k.
//! Если паспорт даёт точки «стоповый момент, стоповый ток, скорость холостого хода»,
//! константы вычисляются из них: R = V/I_стоп, K_t = T_стоп/(I_стоп − I₀),
//! K_e = (V − R·I₀)/ω₀, и прямая проходит точно через паспортные точки.
//!
//! Если у QDD опубликованы пары «номинальный момент/ток» и «пиковый момент/ток», связь
//! момента с током берётся кусочно-линейной через эти пары: так учитывается падение
//! K_t на пике, которое видно в самих паспортах CubeMars.
//!
//! Нагрев: R растёт по α_Cu, K_t падает по коэффициенту магнита. При температуре ниже
//! 25 °C выигрыш не засчитывается. Насыщение, пульсации момента, ослабление поля и
//! коммутация не моделируются.
//!
//! У шагового двигателя график pull-out опубликован для конкретного полушагового
//! режима (24 В, 2 А). Для произвольного драйвера и напряжения используется
//! верхняя граница по импедансу обмотки.

use crate::spec::{
    ALPHA_CU, FERRITE_KT_COEFF_PER_K, NDFEB_KT_COEFF_PER_K, Provenance, RPM, Spec, T_REF_C,
};

/// Тип двигателя — для совместимости с драйвером.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MotorType {
    Bldc,
    BrushedDc,
    BipolarStepper,
}

impl MotorType {
    pub fn title(self) -> &'static str {
        match self {
            MotorType::Bldc => "бесколлекторный",
            MotorType::BrushedDc => "коллекторный",
            MotorType::BipolarStepper => "шаговый биполярный",
        }
    }
}

/// Класс привода для выпадающих списков.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ActuatorKind {
    GearedBldc,
    Qdd,
    SmartServo,
    HobbyServo,
    BrushedGearmotor,
    Stepper,
}

impl ActuatorKind {
    pub fn motor_type(self) -> MotorType {
        match self {
            ActuatorKind::GearedBldc | ActuatorKind::Qdd => MotorType::Bldc,
            ActuatorKind::SmartServo
            | ActuatorKind::HobbyServo
            | ActuatorKind::BrushedGearmotor => MotorType::BrushedDc,
            ActuatorKind::Stepper => MotorType::BipolarStepper,
        }
    }

    pub fn title(self) -> &'static str {
        match self {
            ActuatorKind::GearedBldc => "бесколлекторный с редуктором",
            ActuatorKind::Qdd => "QDD-привод",
            ActuatorKind::SmartServo => "умный сервопривод",
            ActuatorKind::HobbyServo => "хобби-серво",
            ActuatorKind::BrushedGearmotor => "коллекторный мотор-редуктор",
            ActuatorKind::Stepper => "шаговый двигатель",
        }
    }
}

/// Материал магнитов: от него зависит падение K_t при нагреве.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Magnet {
    NdFeB,
    Ferrite,
    Unknown,
}

impl Magnet {
    pub fn kt_coeff_per_k(self) -> f64 {
        match self {
            Magnet::NdFeB => NDFEB_KT_COEFF_PER_K,
            Magnet::Ferrite | Magnet::Unknown => FERRITE_KT_COEFF_PER_K,
        }
    }
}

/// Можно ли провернуть выходной вал без питания.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Backdrive {
    /// Проворачивается; момент проворота, если опубликован.
    Reversible { torque_nm: Spec },
    /// Самоторможение подтверждено документом.
    SelfLocking { source: &'static str },
    /// Производитель не сообщает.
    Unknown,
}

/// Тормоз. Если производитель тормоз не заявляет, считаем, что его нет.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Brake {
    None,
    /// Тормоз с гарантированным удерживающим моментом при снятии питания.
    Holding {
        torque_nm: Spec,
    },
}

/// Паспортная точка при конкретном напряжении (момент и скорость — на выходе).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VoltagePoint {
    pub voltage_v: f64,
    pub stall_torque_nm: f64,
    pub no_load_rpm: f64,
    pub stall_current_a: Option<f64>,
}

/// Привод из каталога. Все моменты, скорости и токи — как в паспорте; момент и скорость
/// — на выходном валу, если не сказано иное.
#[derive(Clone, Debug)]
pub struct Actuator {
    pub id: &'static str,
    pub name: &'static str,
    pub kind: ActuatorKind,
    /// Короткий текст «что это» для интерфейса.
    pub what_is: &'static str,

    pub voltage_nominal_v: Spec,
    pub voltage_min_v: Spec,
    pub voltage_max_v: Spec,

    /// Длительный момент на выходе, Н·м.
    pub rated_torque_nm: Spec,
    /// Допустимый пиковый момент на выходе, Н·м (безопасное значение).
    pub peak_torque_nm: Spec,
    /// Скорость холостого хода на выходе при номинальном напряжении, об/мин.
    pub no_load_speed_rpm: Spec,
    /// Скорость в номинальной точке на выходе при номинальном напряжении, об/мин.
    pub rated_speed_rpm: Spec,
    pub no_load_current_a: Spec,
    pub stall_current_a: Spec,
    pub rated_current_a: Spec,
    pub peak_current_a: Spec,
    /// Ток покоя электроники привода (умные и хобби-сервы), А.
    pub standby_current_a: Spec,

    /// Сопротивление обмотки (у BLDC — межфазное) при 25 °C, Ом.
    pub resistance_ohm: Spec,
    pub inductance_h: Spec,
    /// K_t на валу двигателя, Н·м/А.
    pub kt_motor_nm_per_a: Spec,
    /// K_e на валу двигателя, В·с/рад.
    pub ke_motor_v_s: Spec,
    /// K_v на валу двигателя, об/мин/В.
    pub kv_rpm_per_v: Spec,
    pub gear_ratio: Spec,
    /// КПД редуктора при прямом потоке мощности.
    pub gear_efficiency: Spec,
    /// Инерция ротора, кг·м².
    pub rotor_inertia_kg_m2: Spec,
    /// Инерция входа редуктора, кг·м².
    pub gear_input_inertia_kg_m2: Spec,
    pub mass_kg: Spec,

    /// Двухузловая тепловая модель «обмотка → корпус → среда».
    pub r_th_winding_housing_k_per_w: Spec,
    pub r_th_housing_ambient_k_per_w: Spec,
    pub tau_winding_s: Spec,
    pub tau_housing_s: Spec,
    /// Полное тепловое сопротивление, если паспорт даёт только предельный перегрев.
    pub r_th_total_k_per_w: Spec,
    pub max_winding_temp_c: Spec,
    pub min_ambient_c: Spec,
    pub max_ambient_c: Spec,

    pub backdrive: Backdrive,
    pub brake: Brake,
    pub magnet: Magnet,
    /// Сколько обмоток греется одновременно (у шагового — две).
    pub heated_phases: f64,
    /// Число пар полюсов шагового двигателя (1.8° → 50).
    pub stepper_pole_pairs: Option<f64>,

    /// Паспортные точки по напряжениям, по возрастанию напряжения.
    pub voltage_points: &'static [VoltagePoint],
    /// Встроенный драйвер: id из каталога драйверов.
    pub builtin_driver: Option<&'static str>,
    pub sources: &'static [&'static str],
    /// Что в данных ненадёжно.
    pub caveats: &'static [&'static str],
}

impl Actuator {
    /// Заготовка для каталога: все числа «не опубликовано».
    pub fn blank(
        id: &'static str,
        name: &'static str,
        kind: ActuatorKind,
        what_is: &'static str,
    ) -> Self {
        Self {
            id,
            name,
            kind,
            what_is,
            voltage_nominal_v: Spec::NONE,
            voltage_min_v: Spec::NONE,
            voltage_max_v: Spec::NONE,
            rated_torque_nm: Spec::NONE,
            peak_torque_nm: Spec::NONE,
            no_load_speed_rpm: Spec::NONE,
            rated_speed_rpm: Spec::NONE,
            no_load_current_a: Spec::NONE,
            stall_current_a: Spec::NONE,
            rated_current_a: Spec::NONE,
            peak_current_a: Spec::NONE,
            standby_current_a: Spec::NONE,
            resistance_ohm: Spec::NONE,
            inductance_h: Spec::NONE,
            kt_motor_nm_per_a: Spec::NONE,
            ke_motor_v_s: Spec::NONE,
            kv_rpm_per_v: Spec::NONE,
            gear_ratio: Spec::NONE,
            gear_efficiency: Spec::NONE,
            rotor_inertia_kg_m2: Spec::NONE,
            gear_input_inertia_kg_m2: Spec::NONE,
            mass_kg: Spec::NONE,
            r_th_winding_housing_k_per_w: Spec::NONE,
            r_th_housing_ambient_k_per_w: Spec::NONE,
            tau_winding_s: Spec::NONE,
            tau_housing_s: Spec::NONE,
            r_th_total_k_per_w: Spec::NONE,
            max_winding_temp_c: Spec::NONE,
            min_ambient_c: Spec::NONE,
            max_ambient_c: Spec::NONE,
            backdrive: Backdrive::Unknown,
            brake: Brake::None,
            magnet: Magnet::Unknown,
            heated_phases: 1.0,
            stepper_pole_pairs: None,
            voltage_points: &[],
            builtin_driver: None,
            sources: &[],
            caveats: &[],
        }
    }

    /// Все числовые поля с именами — для проверки на NaN/∞ и отрицательные значения.
    pub fn numeric_specs(&self) -> Vec<(&'static str, Spec)> {
        let mut v = vec![
            ("voltage_nominal_v", self.voltage_nominal_v),
            ("voltage_min_v", self.voltage_min_v),
            ("voltage_max_v", self.voltage_max_v),
            ("rated_torque_nm", self.rated_torque_nm),
            ("peak_torque_nm", self.peak_torque_nm),
            ("no_load_speed_rpm", self.no_load_speed_rpm),
            ("rated_speed_rpm", self.rated_speed_rpm),
            ("no_load_current_a", self.no_load_current_a),
            ("stall_current_a", self.stall_current_a),
            ("rated_current_a", self.rated_current_a),
            ("peak_current_a", self.peak_current_a),
            ("standby_current_a", self.standby_current_a),
            ("resistance_ohm", self.resistance_ohm),
            ("inductance_h", self.inductance_h),
            ("kt_motor_nm_per_a", self.kt_motor_nm_per_a),
            ("ke_motor_v_s", self.ke_motor_v_s),
            ("kv_rpm_per_v", self.kv_rpm_per_v),
            ("gear_ratio", self.gear_ratio),
            ("gear_efficiency", self.gear_efficiency),
            ("rotor_inertia_kg_m2", self.rotor_inertia_kg_m2),
            ("gear_input_inertia_kg_m2", self.gear_input_inertia_kg_m2),
            ("mass_kg", self.mass_kg),
            (
                "r_th_winding_housing_k_per_w",
                self.r_th_winding_housing_k_per_w,
            ),
            (
                "r_th_housing_ambient_k_per_w",
                self.r_th_housing_ambient_k_per_w,
            ),
            ("tau_winding_s", self.tau_winding_s),
            ("tau_housing_s", self.tau_housing_s),
            ("r_th_total_k_per_w", self.r_th_total_k_per_w),
        ];
        if let Backdrive::Reversible { torque_nm } = self.backdrive {
            v.push(("backdrive_torque_nm", torque_nm));
        }
        if let Brake::Holding { torque_nm } = self.brake {
            v.push(("brake_torque_nm", torque_nm));
        }
        v
    }

    /// Температуры могут быть отрицательными, поэтому проверяются отдельно.
    pub fn temperature_specs(&self) -> [(&'static str, Spec); 3] {
        [
            ("max_winding_temp_c", self.max_winding_temp_c),
            ("min_ambient_c", self.min_ambient_c),
            ("max_ambient_c", self.max_ambient_c),
        ]
    }

    /// Приведённая к выходу инерция (ротор + известный вклад входа редуктора)·N², кг·м².
    /// Если инерция входа не опубликована, результат содержит только вклад ротора.
    pub fn reflected_inertia_kg_m2(&self) -> Option<f64> {
        let j = self.rotor_inertia_kg_m2.get()?;
        let n = self.gear_ratio.get()?;
        let j_in = match self.gear_input_inertia_kg_m2.value {
            Some(_) => self.gear_input_inertia_kg_m2.get()?,
            // Ограничиваем результат вкладом опубликованной инерции ротора.
            None => 0.0,
        };
        let reflected = (j + j_in) * n * n;
        (reflected.is_finite() && j >= 0.0 && j_in >= 0.0 && n > 0.0).then_some(reflected)
    }

    /// Паспортная точка при напряжении `v`: между точками — линейно; ниже нижней —
    /// меньшее из пропорционального снижения и продолжения прямой; выше верхней —
    /// верхняя точка (лишнее напряжение в запас момента не засчитывается).
    pub fn point_at(&self, v: f64) -> Option<VoltagePoint> {
        let p = self.voltage_points;
        let first = *p.first()?;
        let last = *p.last()?;
        if !(v.is_finite() && v > 0.0) {
            return None;
        }
        if v >= last.voltage_v {
            return Some(last);
        }
        if v <= first.voltage_v {
            let s = v / first.voltage_v;
            let mut q = VoltagePoint {
                voltage_v: v,
                stall_torque_nm: first.stall_torque_nm * s,
                no_load_rpm: first.no_load_rpm * s,
                stall_current_a: first.stall_current_a.map(|i| i * s),
            };
            if p.len() >= 2 {
                let e = lerp_point(p[0], p[1], v);
                q.stall_torque_nm = q.stall_torque_nm.min(e.stall_torque_nm).max(0.0);
                q.no_load_rpm = q.no_load_rpm.min(e.no_load_rpm).max(0.0);
            }
            return Some(q);
        }
        p.windows(2)
            .find(|w| v >= w[0].voltage_v && v <= w[1].voltage_v)
            .map(|w| lerp_point(w[0], w[1], v))
    }

    /// Рабочая прямая при напряжении `v` и температуре обмотки `winding_c`.
    /// `None`, если данных для неё нет.
    pub fn motor_line(&self, v: f64, winding_c: f64) -> Option<MotorLine> {
        if !(v.is_finite() && v > 0.0 && winding_c.is_finite()) {
            return None;
        }
        let dt = (winding_c - T_REF_C).max(0.0);
        let f_r = 1.0 + ALPHA_CU * dt;
        let f_k = (1.0 - self.magnet.kt_coeff_per_k() * dt).max(0.05);

        if self.kind == ActuatorKind::Stepper {
            let r = self.resistance_ohm.get()?;
            let hold = self.peak_torque_nm.get()?;
            let i_rated = self.rated_current_a.get()?;
            let k_t = hold / i_rated;
            let i_lim = i_rated.min(v / (r * f_r));
            return Some(MotorLine {
                voltage_v: v,
                v_cap: f64::MAX,
                winding_c,
                r_ohm: Some(r * f_r),
                k_e_v_s: 0.0,
                k_t_nm_per_a: Some(k_t),
                i0_a: 0.0,
                magnet_factor: f_k,
                stall_torque_nm: k_t * f_k * i_lim,
                no_load_rad_s: 0.0,
                piecewise: None,
                stepper: Some(StepperLine {
                    inductance_h: self.inductance_h.get()?,
                    pole_pairs: self.stepper_pole_pairs?,
                    rated_current_a: i_rated,
                }),
            });
        }

        let v_cap = self.voltage_max_v.get().unwrap_or(f64::MAX);
        let v = v.min(v_cap);
        if !self.voltage_points.is_empty() {
            let q = self.point_at(v)?;
            let w0 = q.no_load_rpm * RPM;
            let i0 = self.no_load_current_a.get().unwrap_or(0.0);
            return Some(match q.stall_current_a {
                Some(is) if is > i0 => {
                    let r = q.voltage_v / is;
                    let k_t = q.stall_torque_nm / (is - i0);
                    let k_e = (q.voltage_v - r * i0) / w0;
                    MotorLine {
                        voltage_v: q.voltage_v,
                        v_cap: q.voltage_v,
                        winding_c,
                        r_ohm: Some(r * f_r),
                        k_e_v_s: k_e,
                        k_t_nm_per_a: Some(k_t),
                        i0_a: i0,
                        magnet_factor: f_k,
                        stall_torque_nm: (k_t * f_k * (q.voltage_v / (r * f_r) - i0)).max(0.0),
                        no_load_rad_s: ((q.voltage_v - r * f_r * i0) / k_e).max(0.0),
                        piecewise: None,
                        stepper: None,
                    }
                }
                _ => MotorLine {
                    voltage_v: q.voltage_v,
                    v_cap: q.voltage_v,
                    winding_c,
                    r_ohm: None,
                    k_e_v_s: q.voltage_v / w0,
                    k_t_nm_per_a: None,
                    i0_a: 0.0,
                    magnet_factor: f_k,
                    stall_torque_nm: q.stall_torque_nm * f_k / f_r,
                    no_load_rad_s: w0,
                    piecewise: None,
                    stepper: None,
                },
            });
        }

        let r = self.resistance_ohm.get()?;
        let k = self.kt_motor_nm_per_a.get()?;
        let n = self.gear_ratio.get()?;
        let eta = self.gear_efficiency.get().unwrap_or(1.0);
        // Для скорости берём наибольшую из опубликованных констант: меньше скорость.
        let mut ke_m = k;
        if let Some(ke) = self.ke_motor_v_s.get() {
            ke_m = ke_m.max(ke);
        }
        if let Some(kv) = self.kv_rpm_per_v.get() {
            ke_m = ke_m.max(1.0 / (kv * RPM));
        }
        let mut k_e = n * ke_m;
        // Паспортная номинальная точка часто медленнее, чем следует из K_t: берём худшее.
        if let (Some(rpm), Some(i), Some(vn)) = (
            self.rated_speed_rpm.get(),
            self.rated_current_a.get(),
            self.voltage_nominal_v.get(),
        ) && rpm > 0.0
        {
            k_e = k_e.max((vn - r * i) / (rpm * RPM));
        }
        let piecewise = self.published_torque_current();
        let (k_t, i0) = match piecewise {
            Some([(t1, i1), _]) => (t1 / i1, 0.0),
            None => (n * eta * k, self.no_load_current_a.get().unwrap_or(0.0)),
        };
        let r_hot = r * f_r;
        let mut line = MotorLine {
            voltage_v: v,
            v_cap,
            winding_c,
            r_ohm: Some(r_hot),
            k_e_v_s: k_e,
            k_t_nm_per_a: Some(k_t),
            i0_a: i0,
            magnet_factor: f_k,
            stall_torque_nm: 0.0,
            no_load_rad_s: ((v - r_hot * i0) / k_e).max(0.0),
            piecewise,
            stepper: None,
        };
        line.stall_torque_nm = line.torque_for_current(v / r_hot);
        Some(line)
    }

    /// Пары «момент, ток» из паспорта, если опубликованы обе и напрямую производителем.
    fn published_torque_current(&self) -> Option<[(f64, f64); 2]> {
        let direct = |s: Spec| s.provenance == Provenance::Datasheet;
        if !(direct(self.rated_torque_nm)
            && direct(self.rated_current_a)
            && direct(self.peak_torque_nm)
            && direct(self.peak_current_a))
        {
            return None;
        }
        let (t1, i1) = (self.rated_torque_nm.get()?, self.rated_current_a.get()?);
        let (t2, i2) = (self.peak_torque_nm.get()?, self.peak_current_a.get()?);
        (t1 > 0.0 && i1 > 0.0 && t2 > t1 && i2 > i1).then_some([(t1, i1), (t2, i2)])
    }

    /// Доступный момент при скорости `speed_rad_s`, напряжении `v` и температуре
    /// обмотки `winding_c`, с учётом паспортного пика. Для шагового — верхняя граница.
    pub fn available_torque_nm(&self, v: f64, speed_rad_s: f64, winding_c: f64) -> Option<f64> {
        if !speed_rad_s.is_finite() || self.peak_torque_nm.is_broken() {
            return None;
        }
        let line = self.motor_line(v, winding_c)?;
        let t = line.available_torque(speed_rad_s.abs(), v);
        if !t.is_finite() {
            return None;
        }
        Some(match self.peak_torque_nm.get() {
            Some(p) => t.min(p),
            None => t,
        })
    }

    /// Кривая для графика: точки (скорость на выходе, рад/с; момент, Н·м).
    pub fn torque_speed_curve(&self, v: f64, winding_c: f64, points: usize) -> Vec<(f64, f64)> {
        let Some(line) = self.motor_line(v, winding_c) else {
            return Vec::new();
        };
        let n = points.max(2);
        let w_end = if let Some(s) = line.stepper {
            // Частота, на которой индуктивное сопротивление втрое больше активного.
            3.0 * line.r_ohm.unwrap_or(1.0) / (s.pole_pairs * s.inductance_h)
        } else {
            line.no_load_rad_s
        };
        (0..n)
            .map(|j| {
                let w = w_end * j as f64 / (n - 1) as f64;
                let t = line.available_torque(w, v);
                let t = self.peak_torque_nm.get().map_or(t, |p| t.min(p));
                (w, t)
            })
            .collect()
    }
}

fn lerp_point(a: VoltagePoint, b: VoltagePoint, v: f64) -> VoltagePoint {
    let s = (v - a.voltage_v) / (b.voltage_v - a.voltage_v);
    let l = |x: f64, y: f64| x + s * (y - x);
    VoltagePoint {
        voltage_v: v,
        stall_torque_nm: l(a.stall_torque_nm, b.stall_torque_nm),
        no_load_rpm: l(a.no_load_rpm, b.no_load_rpm),
        stall_current_a: match (a.stall_current_a, b.stall_current_a) {
            (Some(x), Some(y)) => Some(l(x, y)),
            _ => None,
        },
    }
}

/// Данные шагового двигателя для верхней границы момента.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StepperLine {
    pub inductance_h: f64,
    pub pole_pairs: f64,
    pub rated_current_a: f64,
}

/// Рабочая прямая привода при заданном напряжении и температуре обмотки.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorLine {
    /// Напряжение, для которого построена прямая, В.
    pub voltage_v: f64,
    /// Выше этого напряжения прирост момента и скорости не засчитывается.
    pub v_cap: f64,
    pub winding_c: f64,
    /// Горячее сопротивление, Ом.
    pub r_ohm: Option<f64>,
    /// K_e на выходе, В·с/рад.
    pub k_e_v_s: f64,
    /// K_t на выходе при 25 °C, Н·м/А.
    pub k_t_nm_per_a: Option<f64>,
    pub i0_a: f64,
    /// Во сколько раз K_t меньше холодного из-за нагрева магнитов.
    pub magnet_factor: f64,
    /// Момент при ω = 0 и напряжении `voltage_v`, горячий, Н·м.
    pub stall_torque_nm: f64,
    /// Скорость холостого хода, рад/с (у шагового не определена — 0).
    pub no_load_rad_s: f64,
    pub piecewise: Option<[(f64, f64); 2]>,
    pub stepper: Option<StepperLine>,
}

impl MotorLine {
    pub(crate) fn is_finite(&self) -> bool {
        [
            self.voltage_v,
            self.v_cap,
            self.winding_c,
            self.k_e_v_s,
            self.i0_a,
            self.magnet_factor,
            self.stall_torque_nm,
            self.no_load_rad_s,
        ]
        .into_iter()
        .all(f64::is_finite)
            && self.r_ohm.is_none_or(|v| v.is_finite() && v > 0.0)
            && self.k_t_nm_per_a.is_none_or(|v| v.is_finite() && v > 0.0)
    }

    /// Ток для момента `tau_abs` ≥ 0 при текущей температуре. `None` — нет K_t.
    pub fn current_for_torque(&self, tau_abs: f64) -> Option<f64> {
        let tau = tau_abs / self.magnet_factor;
        if let Some([(t1, i1), (t2, i2)]) = self.piecewise {
            return Some(if tau <= t1 {
                i1 * tau / t1
            } else {
                i1 + (tau - t1) * (i2 - i1) / (t2 - t1)
            });
        }
        let k = self.k_t_nm_per_a?;
        Some(tau / k + self.i0_a)
    }

    /// Момент при токе `i` с учётом нагрева. Без K_t — 0.
    pub fn torque_for_current(&self, i: f64) -> f64 {
        if i <= 0.0 {
            return 0.0;
        }
        let cold = if let Some([(t1, i1), (t2, i2)]) = self.piecewise {
            if i <= i1 {
                t1 * i / i1
            } else {
                t1 + (i - i1) * (t2 - t1) / (i2 - i1)
            }
        } else if let Some(k) = self.k_t_nm_per_a {
            k * (i - self.i0_a).max(0.0)
        } else {
            0.0
        };
        cold * self.magnet_factor
    }

    fn stepper_impedance(&self, s: StepperLine, w: f64) -> f64 {
        let r = self.r_ohm.unwrap_or(0.0);
        (r * r + (s.pole_pairs * w * s.inductance_h).powi(2)).sqrt()
    }

    /// Доступный момент при скорости `w_abs` и напряжении на зажимах `v`.
    pub fn available_torque(&self, w_abs: f64, v: f64) -> f64 {
        if !(w_abs.is_finite() && v.is_finite() && self.is_finite()) {
            return f64::NAN;
        }
        let v = v.min(self.v_cap).max(0.0);
        if let Some(s) = self.stepper {
            let z = self.stepper_impedance(s, w_abs);
            if !z.is_finite() {
                return f64::NAN;
            }
            let i = s.rated_current_a.min(v / z);
            return self.k_t_nm_per_a.unwrap_or(0.0) * self.magnet_factor * i;
        }
        match self.r_ohm {
            Some(r) => {
                let i = (v - self.k_e_v_s * w_abs) / r;
                if !i.is_finite() {
                    return f64::NAN;
                }
                self.torque_for_current(i)
            }
            None => {
                let s = v / self.voltage_v;
                let (t0, w0) = (self.stall_torque_nm * s, self.no_load_rad_s * s);
                if w0 > 0.0 {
                    t0 * (1.0 - w_abs / w0).max(0.0)
                } else {
                    0.0
                }
            }
        }
    }

    /// Наибольшая скорость при моменте `tau_abs` и напряжении на зажимах `v`.
    pub fn available_speed(&self, tau_abs: f64, v: f64) -> f64 {
        if !(tau_abs.is_finite() && v.is_finite() && self.is_finite()) {
            return f64::NAN;
        }
        let v = v.min(self.v_cap).max(0.0);
        if let Some(s) = self.stepper {
            let Some(i) = self.current_for_torque(tau_abs) else {
                return 0.0;
            };
            if i > s.rated_current_a * (1.0 + 1e-9) {
                return 0.0;
            }
            let r = self.r_ohm.unwrap_or(0.0);
            if i <= 0.0 {
                return f64::MAX;
            }
            let z = v / i;
            return if z <= r {
                0.0
            } else {
                (z * z - r * r).sqrt() / (s.pole_pairs * s.inductance_h)
            };
        }
        match (self.r_ohm, self.current_for_torque(tau_abs)) {
            (Some(r), Some(i)) => ((v - r * i) / self.k_e_v_s).max(0.0),
            _ => {
                let s = v / self.voltage_v;
                let (t0, w0) = (self.stall_torque_nm * s, self.no_load_rad_s * s);
                if t0 > 0.0 {
                    w0 * (1.0 - tau_abs / t0).max(0.0)
                } else {
                    0.0
                }
            }
        }
    }

    /// Напряжение на зажимах, нужное для точки (τ, ω) в любом квадранте. Для шагового —
    /// нижняя граница (без противо-ЭДС). `None` — нет R или K_t.
    pub fn required_voltage(&self, tau: f64, w: f64) -> Option<f64> {
        let r = self.r_ohm?;
        let i = self.current_for_torque(tau.abs())?;
        if let Some(s) = self.stepper {
            return Some(i * self.stepper_impedance(s, w.abs()));
        }
        let e = self.k_e_v_s * w.abs();
        if tau * w >= 0.0 {
            Some(r * i + e)
        } else {
            // Нагрузка ведёт двигатель: ток тормозит, I₀ помогает или мешает — берём худшее.
            let i_lo = (i - 2.0 * self.i0_a).max(0.0);
            Some((e - r * i_lo).abs().max((e - r * i).abs()))
        }
    }
}
