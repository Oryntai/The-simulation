//! Стенд для замкнутых испытаний прошивки `exo-ctrl`: простейший объект
//! управления на хосте и ко-симуляция «прошивка ↔ объект».
//!
//! Объект (модель, допущения — «оценка»):
//! * сустав — одна степень свободы: J·θ̈ = η·N·Kt·i − (m_f·g·r_c + m_груз·g·L)·sin θ
//!   − b·θ̇ + τ_внеш, J = m_f·r_c² + I_cm + m_груз·L² + J_мотора·N²; жёсткие упоры
//!   0° и 150°; КПД редуктора одинаков в обе стороны;
//! * мотор постоянного тока: L·di/dt = V − R·i − Kt·N·θ̇, V = ±заполнение·Vbat
//!   (усреднённая модель H-моста); драйвер выключен → ток сразу 0 (мост в
//!   высоком импедансе, энергия индуктивности уходит через диоды — время спада
//!   не моделируется);
//! * тепло двигателя (если задано [`PlantThermal`]): два узла «обмотка → корпус →
//!   среда», C = τ/R (трактовка maxon), потери в меди R(T)·i², R(T) = R_ref·(1 +
//!   0.00393·(T − T_ref)); датчик ADC2 видит КОРПУС. Без тепловой модели
//!   температура датчика задаётся тестом. Напряжение батареи задаёт тест;
//! * интегрирование — полунеявный Эйлер, шаг ≤ 5 мкс (≪ L/R ≈ 0.67 мс).
//!
//! Шаг ко-симуляции [`Bench::dt`] (по умолчанию 20 мкс): датчики на входы МК →
//! МК исполняет `dt` → выходы МК → объект интегрирует `dt`. Запаздывание
//! обмена — не более одного шага.

use crate::exo::{self, ExoCommand, ExoConfig, ExoTelemetry};
use crate::{Mcu, McuError};

/// Параметры объекта («оценка», согласованы со значениями по умолчанию [`ExoConfig`]).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlantParams {
    /// Сопротивление обмотки, Ом.
    pub r_ohm: f64,
    /// Индуктивность обмотки, Гн.
    pub l_h: f64,
    /// Постоянная момента = постоянная ЭДС (СИ), Н·м/А.
    pub kt: f64,
    pub gear: f64,
    pub efficiency: f64,
    /// Момент инерции ротора, кг·м².
    pub j_motor: f64,
    pub forearm_mass_kg: f64,
    pub forearm_com_m: f64,
    /// Собственный момент инерции предплечья относительно его центра масс, кг·м².
    pub forearm_icm: f64,
    pub hand_dist_m: f64,
    /// Вязкое трение в суставе, Н·м·с/рад.
    pub b_joint: f64,
    pub theta_min_deg: f64,
    pub theta_max_deg: f64,
}

impl PlantParams {
    /// maxon EC 90 flat 260 Вт 48 В (500267) + Harmonic Drive CSG-25-50 (как в
    /// `sim_em::motor::MotorParams::maxon_ec90flat_260w_hd_csg25_50`): R = 0.844 Ом,
    /// L = 1.07 мГн, Kt = 0.231 Н·м/А, N = 50, η = 0.85 («оценка»), инерция ротора и
    /// входа редуктора 5.06·10⁻⁴ + 0.413·10⁻⁴ кг·м².
    pub fn maxon_ec90flat_hd50() -> PlantParams {
        PlantParams {
            r_ohm: 0.844,
            l_h: 1.07e-3,
            kt: 0.231,
            gear: 50.0,
            efficiency: 0.85,
            j_motor: 5.06e-4 + 0.413e-4,
            ..PlantParams::default()
        }
    }
}

/// Тепловая модель двигателя стенда: два узла «обмотка → корпус → среда».
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlantThermal {
    /// «Обмотка → корпус», К/Вт.
    pub r_wh: f64,
    /// «Корпус → среда», К/Вт.
    pub r_ha: f64,
    /// Постоянные времени даташита, с: C_w = τ_w/R_wh, C_h = τ_h/R_ha.
    pub tau_w: f64,
    pub tau_h: f64,
    /// Температура, при которой задано `PlantParams::r_ohm`, °C.
    pub r_ref_c: f64,
    pub ambient_c: f64,
}

/// Температурный коэффициент сопротивления меди, 1/К.
pub const ALPHA_CU: f64 = 0.003_93;

impl PlantThermal {
    /// maxon EC 90 flat 260 Вт: 1.82 К/Вт, 1.78 К/Вт, 54.3 с, 265 с (данные maxon).
    pub fn maxon_ec90flat() -> PlantThermal {
        PlantThermal {
            r_wh: 1.82,
            r_ha: 1.78,
            tau_w: 54.3,
            tau_h: 265.0,
            r_ref_c: 25.0,
            ambient_c: 25.0,
        }
    }
    pub fn c_w(&self) -> f64 {
        self.tau_w / self.r_wh
    }
    pub fn c_h(&self) -> f64 {
        self.tau_h / self.r_ha
    }
}

impl Default for PlantParams {
    fn default() -> Self {
        PlantParams {
            r_ohm: 0.6,
            l_h: 0.4e-3,
            kt: 0.05,
            gear: 100.0,
            efficiency: 0.8,
            j_motor: 5e-6,
            forearm_mass_kg: 1.65,
            forearm_com_m: 0.16,
            forearm_icm: 0.008,
            hand_dist_m: 0.35,
            b_joint: 0.05,
            theta_min_deg: 0.0,
            theta_max_deg: 150.0,
        }
    }
}

const G: f64 = 9.81;

/// Объект: предплечье + мотор + датчики.
#[derive(Clone, Debug)]
pub struct ExoPlant {
    pub p: PlantParams,
    pub payload_kg: f64,
    /// Угол, рад (0 — рука прямая).
    pub theta: f64,
    /// Угловая скорость, рад/с.
    pub omega: f64,
    /// Ток мотора, А.
    pub current: f64,
    pub temp_c: f64,
    pub vbat_v: f64,
    /// Угол удерживается внешней силой (кинематически), °.
    pub hold_deg: Option<f64>,
    /// Внешний момент (рука человека), Н·м.
    pub tau_ext: f64,
    /// Подмена показания датчика тока, А (имитация неисправности).
    pub current_sense_override_a: Option<f64>,
    /// Подмена напряжения датчика угла, В (имитация обрыва).
    pub angle_volts_override: Option<f64>,
    /// Тепловая модель двигателя (`None` — температуру датчика задаёт `temp_c`).
    pub thermal: Option<PlantThermal>,
    /// Температуры обмотки и корпуса, °C (используются при `thermal`).
    pub t_winding_c: f64,
    pub t_housing_c: f64,
}

impl ExoPlant {
    pub fn new(p: PlantParams, theta_deg: f64) -> ExoPlant {
        ExoPlant {
            p,
            payload_kg: 0.0,
            theta: theta_deg.to_radians(),
            omega: 0.0,
            current: 0.0,
            temp_c: 30.0,
            vbat_v: 24.0,
            hold_deg: None,
            tau_ext: 0.0,
            current_sense_override_a: None,
            angle_volts_override: None,
            thermal: None,
            t_winding_c: 30.0,
            t_housing_c: 30.0,
        }
    }

    /// Включить тепловую модель с начальными температурами обмотки и корпуса.
    pub fn with_thermal(mut self, th: PlantThermal, winding_c: f64, housing_c: f64) -> ExoPlant {
        self.thermal = Some(th);
        self.t_winding_c = winding_c;
        self.t_housing_c = housing_c;
        self
    }

    /// Сопротивление обмотки сейчас, Ом (с учётом температуры, если есть тепловая модель).
    pub fn winding_resistance(&self) -> f64 {
        match self.thermal {
            Some(th) => self.p.r_ohm * (1.0 + ALPHA_CU * (self.t_winding_c - th.r_ref_c)),
            None => self.p.r_ohm,
        }
    }

    /// Температура, которую видит датчик ADC2, °C.
    pub fn sensor_temp_c(&self) -> f64 {
        if self.thermal.is_some() {
            self.t_housing_c
        } else {
            self.temp_c
        }
    }

    pub fn theta_deg(&self) -> f64 {
        self.theta.to_degrees()
    }

    /// Приведённый к суставу момент инерции, кг·м².
    pub fn inertia(&self) -> f64 {
        let p = &self.p;
        p.forearm_mass_kg * p.forearm_com_m.powi(2)
            + p.forearm_icm
            + self.payload_kg * p.hand_dist_m.powi(2)
            + p.j_motor * p.gear * p.gear
    }

    /// Момент тяжести (знак: тянет к разгибанию), Н·м.
    pub fn gravity_torque(&self) -> f64 {
        let p = &self.p;
        -(p.forearm_mass_kg * G * p.forearm_com_m + self.payload_kg * G * p.hand_dist_m)
            * self.theta.sin()
    }

    /// Момент мотора на выходе редуктора, Н·м.
    pub fn motor_torque(&self) -> f64 {
        self.p.efficiency * self.p.gear * self.p.kt * self.current
    }

    /// Шаг `dt` с выходами МК. Момент тяжести и инерция считаются раз на шаг
    /// (за ≤ 20 мкс угол меняется на доли тысячной радиана).
    pub fn step(&mut self, duty: f64, flex: bool, enable: bool, dt: f64) {
        let n = (dt / 5e-6 - 1e-9).ceil().max(1.0) as usize;
        let h = dt / n as f64;
        let p = self.p;
        let inv_j = 1.0 / self.inertia();
        let tau_g = self.gravity_torque();
        let torque_per_amp = p.efficiency * p.gear * p.kt;
        let emf_per_omega = p.kt * p.gear;
        let v = if enable {
            (if flex { 1.0 } else { -1.0 }) * duty * self.vbat_v
        } else {
            0.0
        };
        let (lo, hi) = (p.theta_min_deg.to_radians(), p.theta_max_deg.to_radians());
        let r = self.winding_resistance();
        let mut copper_j = 0.0;
        for _ in 0..n {
            if enable {
                self.current += h * (v - r * self.current - emf_per_omega * self.omega) / p.l_h;
            } else {
                self.current = 0.0;
            }
            copper_j += r * self.current * self.current * h;
            if let Some(d) = self.hold_deg {
                self.theta = d.to_radians();
                self.omega = 0.0;
            } else {
                let acc = (torque_per_amp * self.current + tau_g - p.b_joint * self.omega
                    + self.tau_ext)
                    * inv_j;
                self.omega += h * acc;
                self.theta += h * self.omega;
                if self.theta < lo {
                    self.theta = lo;
                    self.omega = self.omega.max(0.0);
                } else if self.theta > hi {
                    self.theta = hi;
                    self.omega = self.omega.min(0.0);
                }
            }
        }
        if let Some(th) = self.thermal {
            // Явный Эйлер на шаге обмена (dt ≤ 100 мкс ≪ τ ≈ 54 с).
            let p_w = copper_j / dt;
            let q_wh = (self.t_winding_c - self.t_housing_c) / th.r_wh;
            let q_ha = (self.t_housing_c - th.ambient_c) / th.r_ha;
            self.t_winding_c += dt * (p_w - q_wh) / th.c_w();
            self.t_housing_c += dt * (q_wh - q_ha) / th.c_h();
        }
    }

    /// Показания датчиков → входы АЦП (по шкалам контракта, с подменами).
    pub fn apply_sensors(&self, mcu: &mut Mcu) {
        let i = self.current_sense_override_a.unwrap_or(self.current);
        exo::write_sensors(mcu, self.theta_deg(), i, self.sensor_temp_c(), self.vbat_v);
        if let Some(v) = self.angle_volts_override {
            mcu.set_adc_volts(exo::pins::ADC_ANGLE, v);
        }
    }
}

/// Ко-симуляция прошивки и объекта.
#[derive(Clone, Debug)]
pub struct Bench {
    pub mcu: Mcu,
    pub plant: ExoPlant,
    /// Шаг обмена, с.
    pub dt: f64,
    /// Принятая телеметрия: (время, пакет).
    pub telemetry: Vec<(f64, ExoTelemetry)>,
    seq: u8,
}

impl Bench {
    /// Загрузить прошивку, записать конфигурацию, подать датчики и сбросить МК.
    pub fn new(elf: &[u8], cfg: &ExoConfig, plant: ExoPlant) -> Result<Bench, McuError> {
        let mut mcu = Mcu::from_elf(elf)?;
        mcu.write_config(&cfg.to_bytes())?;
        plant.apply_sensors(&mut mcu);
        mcu.reset();
        Ok(Bench {
            mcu,
            plant,
            dt: 20e-6,
            telemetry: Vec::new(),
            seq: 0,
        })
    }

    pub fn time(&self) -> f64 {
        self.mcu.time()
    }

    /// Один шаг обмена.
    pub fn step(&mut self) {
        self.plant.apply_sensors(&mut self.mcu);
        // dt конечен и положителен — ошибки быть не может; сбой прошивки виден через mcu.fault().
        let _ = self.mcu.run_for(self.dt);
        let (duty, flex, en) = exo::read_drive(&self.mcu);
        self.plant.step(duty, flex, en, self.dt);
        let t = self.mcu.time();
        for pkt in self.mcu.radio_take_tx() {
            if let Some(tl) = ExoTelemetry::decode(&pkt) {
                self.telemetry.push((t, tl));
            }
        }
    }

    /// Выполнять шаги `seconds` секунд.
    pub fn run(&mut self, seconds: f64) {
        let end = self.time() + seconds;
        while self.time() < end - 1e-12 {
            self.step();
        }
    }

    /// Выполнять шаги, вызывая `f` после каждого; `f` возвращает `false` — стоп.
    pub fn run_while(&mut self, seconds: f64, mut f: impl FnMut(&mut Bench) -> bool) {
        let end = self.time() + seconds;
        while self.time() < end - 1e-12 {
            self.step();
            if !f(self) {
                break;
            }
        }
    }

    /// Отправить команду по радио (номер пакета ставится автоматически).
    pub fn send(&mut self, mut cmd: ExoCommand) -> bool {
        self.seq = self.seq.wrapping_add(1);
        cmd.seq = self.seq;
        self.mcu.radio_deliver(&cmd.encode(), -55)
    }

    /// Последняя телеметрия.
    pub fn last_telemetry(&self) -> Option<&ExoTelemetry> {
        self.telemetry.last().map(|(_, t)| t)
    }

    /// Выходы драйвера сейчас: (заполнение, сгибание, включён).
    pub fn drive(&self) -> (f64, bool, bool) {
        exo::read_drive(&self.mcu)
    }
}
