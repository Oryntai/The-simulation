//! Сценарий «Экзо целиком: прошивка в контуре». Настоящая прошивка (машинный код RISC-V,
//! `firmware/exo-ctrl`) крутится на виртуальном микроконтроллере (`sim-mcu`) и управляет
//! симулированным железом:
//!
//! * мотор с редуктором и нагревом обмотки (`sim-em`: maxon EC 90 flat + Harmonic Drive или
//!   CubeMars AK80-64) через H-мост (усреднённая модель ШИМ);
//! * локоть с массой и инерцией предплечья из анатомической модели (MS-Human-700);
//! * человек (расслаблен / держит позу сам / сопротивляется);
//! * батарея (Li-ion, падение напряжения под током, холод);
//! * связь Bluetooth LE с телефоном (`sim-radio`: потери, задержки, разрыв);
//! * среда: температура воздуха, вода (Архимед + сопротивление + охлаждение мотора).
//!
//! Отказы включаются кнопками: обрыв датчика угла, заклинивание, аварийная кнопка, пропала
//! связь, зависание прошивки (проверка сторожевого таймера).

use std::path::PathBuf;
use std::sync::{Arc, Mutex};

/// Результат фоновой сборки прошивки (заполняется потоком сборки).
type BuildSlot = Arc<Mutex<Option<Result<String, String>>>>;

use sim_core::{Pose, Shape, Vec3};
use sim_em::motor::{Motor, MotorParams};
use sim_mcu::Mcu;
use sim_mcu::exo::{
    ExoCommand, ExoConfig, ExoDiag, ExoMode, ExoTelemetry, fault, pins, read_drive, sensors,
};
use sim_radio::channel::{Dir, PacketChannel, Protocol};
use sim_radio::link::packet_error_rate;
use sim_radio::pathloss::Surroundings;
use sim_radio::phy::Phy;
use sim_radio::wearable::WearableLink;

use crate::battery_pack::Pack;
use crate::elbow_plant::{ElbowLoad, ElbowParams, ElbowPlant, Obstacle};
use crate::human_limits::{ContactLimit, CONTACT_FOREARM, CONTACT_HAND, TRANSIENT_FACTOR, TRANSIENT_WINDOW_S};
use crate::scenario::*;

/// Шаг совместной симуляции: 25 мкс — четыре обмена за период контура тока прошивки
/// (10 кГц), чтобы ШИМ, обновлённый в прерывании, доходил до мотора без лишней задержки.
const DT: f64 = 25e-6;
/// Запись истории — каждые 2 мс.
const HIST_EVERY: u64 = 80;
/// Период команд телефона.
const CMD_PERIOD: f64 = 0.05;
/// Расстояние от оси локтя до манжеты на предплечье, м.
pub const CUFF_M: f64 = 0.15;
/// Ток электроники (МК, радио, датчики), А.
const ELECTRONICS_A: f64 = 0.12;

const MOTORS: [&str; 2] = [
    "maxon EC 90 flat 48 В + Harmonic Drive 50:1",
    "CubeMars AK80-64 (мотор-редуктор 64:1)",
];
const HUMANS: [&str; 3] = [
    "человек держит позу сам (мышцы)",
    "человек расслаблен",
    "человек сопротивляется (тянет разогнуть)",
];
const MODES: [&str; 3] = ["выключено", "помощь весу", "держать угол"];

pub fn firmware_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../firmware")
}

pub fn exo_ctrl_elf() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../assets/firmware/exo-ctrl.elf")
}

/// Поля конфигурации прошивки для текстового редактора: (имя, описание).
macro_rules! cfg_fields {
    ($m:ident) => {
        $m!(
            f32 kt_nm_per_a "Kt мотора, Н·м/А",
            f32 gear_ratio "передаточное число",
            f32 gear_efficiency "КПД редуктора",
            f32 i_max_a "предел тока, А",
            f32 i_trip_a "ток аварийного отключения, А",
            f32 cells_series "элементов батареи последовательно",
            f32 forearm_mass_kg "масса предплечья с кистью, кг",
            f32 forearm_com_m "центр масс предплечья от локтя, м",
            f32 hand_dist_m "хват от локтя, м",
            f32 angle_min_deg "мягкий предел разгибания, °",
            f32 angle_max_deg "мягкий предел сгибания, °",
            f32 temp_derate_c "корпус: начало снижения тока, °C",
            f32 temp_off_c "корпус: отключение, °C",
            f32 pos_kp "угол: П, А/°",
            f32 pos_ki "угол: И, А/(°·с)",
            f32 pos_kd "угол: Д, А/(°/с)",
            f32 cur_kp "ток: П, В/А",
            f32 cur_ki "ток: И, В/(А·с)",
            f32 cur_kaw "ток: антинасыщение",
            f32 v_bus_nominal "номинал шины, В",
            u32 wdt_timeout_ms "сторожевой таймер, мс",
            f32 motor_r_ohm "сопротивление обмотки, Ом",
            f32 motor_r_ref_c "при температуре, °C",
            f32 rth_wh_k_per_w "R_th обмотка→корпус, К/Вт",
            f32 tau_w_s "τ обмотки, с",
            f32 t_winding_max_c "предел обмотки, °C",
            f32 stall_time_s "время до отключения при заклинивании, с",
            f32 stall_pos_err_deg "заклинивание: ошибка угла не меньше, °",
            f32 stall_ramp_s "заклинивание: плавный спад тока, с",
            f32 speed_limit_dps "предел скорости руки, °/с (0 — без предела)",
            f32 contact_torque_nm "контакт: лишний момент, Н·м (0 — выключено)"
        )
    };
}

fn config_to_text(c: &ExoConfig) -> String {
    let mut out = String::from("# Конфигурация прошивки exo-ctrl: имя = значение  # пояснение\n");
    macro_rules! emit {
        ($($t:ident $name:ident $desc:literal),*) => {
            $( out += &format!("{} = {}  # {}\n", stringify!($name), c.$name, $desc); )*
        };
    }
    cfg_fields!(emit);
    out
}

fn config_from_text(base: &ExoConfig, text: &str) -> Result<ExoConfig, String> {
    let mut c = *base;
    for (ln, line) in text.lines().enumerate() {
        let line = line.split('#').next().unwrap_or("").trim();
        if line.is_empty() {
            continue;
        }
        let (k, v) = line
            .split_once('=')
            .ok_or_else(|| format!("строка {}: нужно «имя = значение»", ln + 1))?;
        let (k, v) = (k.trim(), v.trim());
        let mut known = false;
        macro_rules! set {
            ($($t:ident $name:ident $desc:literal),*) => {
                $( if k == stringify!($name) {
                    c.$name = v.parse::<$t>().map_err(|_| format!("строка {}: «{v}» — не число", ln + 1))?;
                    known = true;
                } )*
            };
        }
        cfg_fields!(set);
        if !known {
            return Err(format!("строка {}: неизвестное поле «{k}»", ln + 1));
        }
    }
    Ok(c)
}

#[derive(Clone, Copy, Debug, Default)]
struct Faults {
    sensor_break: bool,
    jam: bool,
    estop: bool,
    radio_off: bool,
}

/// Что произошло в контакте с препятствием за прогон.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ContactLog {
    pub first_touch_s: Option<f64>,
    /// Скорость точки контакта в момент касания, м/с.
    pub impact_speed_m_s: f64,
    /// Наибольшая сила за всё время, Н.
    pub peak_n: f64,
    /// Наибольшая сила позже окна удара 0.5 с: это уже зажим, Н.
    pub peak_late_n: f64,
}

/// Один отсчёт эталонной записи контура. Запись только читает состояние.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LoopSample {
    pub t: f64,
    pub theta: f64,
    pub omega: f64,
    pub current_a: f64,
    pub motor_v: f64,
    pub duty: f64,
    pub joint_torque: f64,
    pub human_torque: f64,
    pub stop_torque: f64,
    pub contact_n: f64,
    pub battery_v: f64,
    pub winding_c: f64,
    pub housing_c: f64,
    pub driver_on: bool,
    /// Флаги отказов, которые прошивка уже сообщила по радио.
    pub reported_faults: u16,
}

impl LoopSample {
    /// Все величины отсчёта, в порядке столбцов эталонного лога.
    pub fn values(&self) -> [f64; 15] {
        [
            self.t,
            self.theta,
            self.omega,
            self.current_a,
            self.motor_v,
            self.duty,
            self.joint_torque,
            self.human_torque,
            self.stop_torque,
            self.contact_n,
            self.battery_v,
            self.winding_c,
            self.housing_c,
            f64::from(u8::from(self.driver_on)),
            f64::from(self.reported_faults),
        ]
    }
}

/// Запись прогона: редкие отсчёты для сравнения и отпечаток всех шагов для побитовой сверки.
#[derive(Clone, Debug)]
pub struct LoopTrace {
    every: u64,
    pub steps: u64,
    /// FNV-1a по битам всех величин каждого шага.
    pub hash: u64,
    pub samples: Vec<LoopSample>,
}

impl LoopTrace {
    fn new(every: u64) -> Self {
        Self {
            every: every.max(1),
            steps: 0,
            hash: 0xcbf2_9ce4_8422_2325,
            samples: Vec::new(),
        }
    }

    fn record(&mut self, sample: LoopSample) {
        for value in sample.values() {
            for byte in value.to_bits().to_le_bytes() {
                self.hash = (self.hash ^ u64::from(byte)).wrapping_mul(0x0000_0100_0000_01b3);
            }
        }
        if self.steps.is_multiple_of(self.every) {
            self.samples.push(sample);
        }
        self.steps += 1;
    }
}

pub struct ExoLoop {
    trace: Option<LoopTrace>,
    mcu: Option<Mcu>,
    fw_error: Option<String>,
    cfg: ExoConfig,
    /// Конфигурация, заданная руками в редакторе (иначе — расчёт под мотор и руку).
    user_cfg: Option<ExoConfig>,
    motor_choice: usize,
    /// Элементов последовательно в батарее (6 — под делитель прошивки 1:11; 12 — ошибка платы).
    pack_s: usize,
    motor: Motor,
    plant: ElbowPlant,
    pack: Pack,
    channel: PacketChannel,
    // телефон
    mode: usize,
    assist_pct: f64,
    target_deg: f64,
    phone_payload: f64,
    seq: u8,
    next_cmd: f64,
    last_tel: Option<ExoTelemetry>,
    tel_count: u64,
    // мир
    payload: f64,
    human: usize,
    human_target: f64,
    human_integ: f64,
    human_torque: f64,
    ambient_c: f64,
    water: bool,
    distance: f64,
    body_blocks: bool,
    /// Связь с телефоном: false — Bluetooth LE, true — Wi-Fi.
    wifi: bool,
    faults: Faults,
    // учёт
    t: f64,
    clock: FixedClock,
    hist: History,
    log: Vec<String>,
    estop_at: Option<f64>,
    estop_reaction: Option<f64>,
    stop_hits: u64,
    max_cuff_force: f64,
    /// Сколько секунд подряд большой ток без движения (заклинивание).
    stall_s: f64,
    /// Наибольшая температура обмотки за прогон.
    max_winding_c: f64,
    obstacle: Option<Obstacle>,
    /// Пункт списка «Препятствие»: 0 — нет, 1 — в предплечье, 2 — в кисть.
    obstacle_choice: usize,
    contact: ContactLog,
    build_rx: Option<BuildSlot>,
    epoch: u64,
}

impl Default for ExoLoop {
    fn default() -> Self {
        let params =
            ElbowParams::from_anatomy().unwrap_or_else(|_| ElbowParams::winter(75.0, 1.75));
        let mut s = Self {
            trace: None,
            mcu: None,
            fw_error: None,
            cfg: ExoConfig::default(),
            user_cfg: None,
            motor_choice: 0,
            pack_s: 6,
            motor: Motor::new(MotorParams::maxon_ec90flat_260w_hd_csg25_50()),
            plant: ElbowPlant::new(params, 90f64.to_radians()),
            pack: Pack::new(6, 2, 25.0),
            channel: PacketChannel::new(Protocol::ble_default(), 1),
            mode: 1,
            assist_pct: 70.0,
            target_deg: 90.0,
            phone_payload: 10.0,
            seq: 0,
            next_cmd: 0.0,
            last_tel: None,
            tel_count: 0,
            payload: 10.0,
            human: 0,
            human_target: 90.0,
            human_integ: 0.0,
            human_torque: 0.0,
            ambient_c: 22.0,
            water: false,
            distance: 2.0,
            body_blocks: false,
            wifi: false,
            faults: Faults::default(),
            t: 0.0,
            clock: FixedClock::new(DT),
            hist: History::new(9, 30.0),
            log: Vec::new(),
            estop_at: None,
            estop_reaction: None,
            stop_hits: 0,
            max_cuff_force: 0.0,
            stall_s: 0.0,
            max_winding_c: 0.0,
            obstacle: None,
            obstacle_choice: 0,
            contact: ContactLog::default(),
            build_rx: None,
            epoch: 0,
        };
        s.reset();
        s
    }
}

impl ExoLoop {
    fn motor_params(&self) -> MotorParams {
        if self.motor_choice == 0 {
            MotorParams::maxon_ec90flat_260w_hd_csg25_50()
        } else {
            MotorParams::cubemars_ak80_64()
        }
    }

    /// Конфигурация прошивки под выбранный мотор, батарею и руку. Усиления — ручной расчёт:
    /// контур тока — компенсация полюса обмотки (kp = L·ωc, ki = R·ωc, ωc = 2π·500 Гц);
    /// контур угла — ПИД под суммарную инерцию (предплечье + груз + ротор через редуктор)
    /// на полосу 2 Гц с затуханием 0.8.
    fn make_config(&self, test_hang_ms: u32) -> ExoConfig {
        let p = self.motor_params();
        let kt = p.torque_constant_nm_per_a;
        let n = p.gear.ratio;
        let eta = p.gear.efficiency_forward;
        let tpa = kt * n * eta;
        let wc = 2.0 * std::f64::consts::PI * 500.0;
        let j = self.plant.inertia(self.motor.reflected_inertia());
        let wn = 2.0 * std::f64::consts::PI * 2.0;
        let kp_tau = j * wn * wn; // Н·м/рад
        let kd_tau = 2.0 * 0.8 * wn * j; // Н·м·с/рад
        let deg = std::f64::consts::PI / 180.0;
        let i_cont = p
            .datasheet
            .nominal_current_a
            .or(p.datasheet.rated_current_a)
            .unwrap_or(5.0);
        // Скорость выбираем так, чтобы удар кистью о препятствие F = ω·√(k·J) остался на 25%
        // ниже предела ISO/TS 15066 при инерции привода, руки и груза.
        let hit_n = 0.75 * TRANSIENT_FACTOR * CONTACT_HAND.quasi_static_n;
        let safe_speed = (hit_n / (CONTACT_HAND.stiffness_n_per_m * j).sqrt()).to_degrees();
        ExoConfig {
            kt_nm_per_a: kt as f32,
            gear_ratio: n as f32,
            gear_efficiency: eta as f32,
            i_max_a: (2.0 * i_cont).min(p.max_current_a) as f32,
            i_trip_a: (3.0 * i_cont).min(1.4 * p.max_current_a) as f32,
            cells_series: self.pack.series as f32,
            forearm_mass_kg: self.plant.p.mass as f32,
            forearm_com_m: self.plant.p.com as f32,
            hand_dist_m: self.plant.p.hand as f32,
            angle_min_deg: 5.0,
            angle_max_deg: (self.plant.p.max_angle.to_degrees() - 5.0) as f32,
            temp_derate_c: 70.0,
            temp_off_c: 90.0,
            pos_kp: (kp_tau * deg / tpa) as f32,
            pos_ki: (0.5 * kp_tau * deg / tpa) as f32,
            pos_kd: (kd_tau * deg / tpa) as f32,
            cur_kp: (p.inductance_h * wc) as f32,
            cur_ki: (p.resistance_ohm * wc) as f32,
            cur_kaw: 2000.0,
            v_bus_nominal: self.pack.nominal_voltage() as f32,
            wdt_timeout_ms: 50,
            test_hang_ms,
            // Тепловая защита обмотки (I²t-модель в прошивке) — по паспорту мотора.
            motor_r_ohm: p.resistance_ohm as f32,
            motor_r_ref_c: p.resistance_ref_c as f32,
            rth_wh_k_per_w: p.thermal.r_winding_housing_k_per_w as f32,
            tau_w_s: p.thermal.tau_winding_s as f32,
            t_winding_max_c: p.thermal.max_winding_temp_c as f32,
            stall_time_s: 1.5,
            stall_pos_err_deg: 3.0,
            stall_ramp_s: 0.2,
            speed_limit_dps: safe_speed.min(40.0) as f32,
            contact_torque_nm: 14.0,
        }
    }

    fn load_firmware(&mut self, test_hang_ms: u32) {
        self.cfg = match self.user_cfg {
            Some(c) => ExoConfig { test_hang_ms, ..c },
            None => self.make_config(test_hang_ms),
        };
        let bytes = std::fs::read(exo_ctrl_elf()).or_else(|e| {
            // Файл не найден — берём копию, встроенную в sim-mcu при сборке.
            let b = sim_mcu::firmware::exo_ctrl_elf();
            if b.is_empty() { Err(e) } else { Ok(b.to_vec()) }
        });
        match bytes {
            Ok(bytes) => match Mcu::from_elf(&bytes) {
                Ok(mut m) => {
                    if let Err(e) = m.write_config(&self.cfg.to_bytes()) {
                        self.fw_error = Some(format!("конфигурация не записана: {e}"));
                    }
                    m.reset();
                    self.mcu = Some(m);
                    self.fw_error = None;
                }
                Err(e) => self.fw_error = Some(format!("прошивка не загрузилась: {e}")),
            },
            Err(e) => self.fw_error = Some(format!("нет файла {}: {e}", exo_ctrl_elf().display())),
        }
    }

    fn phy(&self) -> Phy {
        if self.wifi {
            Phy::WifiN20Mcs0
        } else {
            Phy::Ble1M
        }
    }

    fn link(&self) -> WearableLink {
        WearableLink {
            phy: self.phy(),
            tx_dbm: if self.wifi { 15.0 } else { 9.0 },
            distance_m: self.distance,
            surroundings: Surroundings::IndoorLos,
            body_blocks: self.body_blocks,
            medium: if self.water {
                sim_radio::dielectric::Medium::FreshWater {
                    t_c: self.ambient_c,
                }
            } else {
                sim_radio::dielectric::Medium::Air
            },
            depth_m: if self.water { 0.1 } else { 0.0 },
            wet_detuning_db: 0.0,
        }
    }

    fn log(&mut self, s: String) {
        self.log.push(format!("{:8.3} с  {s}", self.t));
    }

    fn exo_mode(&self) -> ExoMode {
        match self.mode {
            1 => ExoMode::GravityAssist,
            2 => ExoMode::PositionTrack,
            _ => ExoMode::Off,
        }
    }

    fn step(&mut self) {
        let Some(mcu) = self.mcu.as_mut() else {
            return;
        };
        let theta_deg = self.plant.theta.to_degrees();
        // 1. Датчики → входы МК.
        let angle_v = if self.faults.sensor_break {
            0.0
        } else {
            sensors::angle_volts(theta_deg.clamp(0.0, 150.0))
        };
        mcu.set_adc_volts(pins::ADC_ANGLE, angle_v);
        mcu.set_adc_volts(
            pins::ADC_CURRENT,
            sensors::current_volts(self.motor.current()),
        );
        mcu.set_adc_volts(
            pins::ADC_TEMP,
            sensors::temp_volts(self.motor.housing_temp_c()),
        );
        let vbat = self.pack.terminal_voltage();
        mcu.set_adc_volts(pins::ADC_VBAT, sensors::battery_volts(vbat));
        mcu.set_gpio_input(pins::GPIO_ESTOP, !self.faults.estop);
        // 2. Прошивка работает один тик.
        if let Err(e) = mcu.run_for(DT) {
            let msg = format!("МК остановился: {e}");
            if self.fw_error.as_deref() != Some(msg.as_str()) {
                self.fw_error = Some(msg.clone());
                self.log.push(format!("{:8.3} с  {msg}", self.t));
            }
        }
        let uart = mcu.uart_take();
        // 3. H-мост: усреднённое напряжение на моторе.
        let (duty, dir_flex, enabled) = read_drive(mcu);
        let i = self.motor.current();
        let v_motor = if enabled {
            duty.clamp(0.0, 1.0) * vbat * if dir_flex { 1.0 } else { -1.0 }
        } else {
            // Драйвер выключен: ток гаснет через обратные диоды ключей в батарею
            // (напряжение −sign(i)·Vbat), пока не дойдёт до нуля; дальше ток не течёт
            // (обмотка разомкнута). Чтобы не «дребезжать» вокруг нуля на шаге, подбираем
            // напряжение, при котором ток за шаг ровно гаснет, если диоды это позволяют.
            let p = self.motor_params();
            let r = self.motor.winding_resistance_ohm();
            let tau = p.inductance_h / r;
            let e = self.motor.back_emf_v();
            let k = (-DT / tau).exp();
            let i_inf = -i * k / (1.0 - k);
            let v_zero = e + r * i_inf;
            if v_zero.abs() <= vbat {
                v_zero
            } else {
                -i.signum() * vbat
            }
        };
        if let Some(at) = self.estop_at
            && self.estop_reaction.is_none()
            && !enabled
        {
            self.estop_reaction = Some(self.t + DT - at);
        }
        // 4. Мотор и сустав.
        self.motor.step_electrical(v_motor, self.plant.omega, DT);
        if !enabled && self.motor.current().abs() < 1e-3 {
            // Не даём численному «звону» вокруг нуля.
        }
        let tau_exo = self.motor.joint_torque();
        let hold = self.plant.holding_torque(self.plant.theta);
        self.human_torque = match self.human {
            0 => {
                // Человек держит позу: ПИ-Д мышц с ограничением силы (60 Н·м — средний мужчина).
                let e = (self.human_target - theta_deg).to_radians();
                self.human_integ = (self.human_integ + e * DT).clamp(-2.0, 2.0);
                (150.0 * e + 300.0 * self.human_integ - 8.0 * self.plant.omega).clamp(-60.0, 60.0)
            }
            1 => 0.0,
            _ => -20.0,
        };
        let _ = hold;
        self.plant.step(
            DT,
            tau_exo + self.human_torque,
            self.motor.reflected_inertia(),
        );
        if self.plant.stop_torque.abs() > 1.0 {
            self.stop_hits += 1;
        }
        let force = self.plant.contact_force;
        if force > 0.0 {
            if self.contact.first_touch_s.is_none() {
                self.contact.first_touch_s = Some(self.t);
                let lever = self.plant.obstacle.map_or(0.0, |o| o.lever);
                self.contact.impact_speed_m_s = (self.plant.omega * lever).abs();
            }
            self.contact.peak_n = self.contact.peak_n.max(force);
            if self.contact.first_touch_s.is_some_and(|t0| self.t - t0 > TRANSIENT_WINDOW_S) {
                self.contact.peak_late_n = self.contact.peak_late_n.max(force);
            }
        }
        self.max_cuff_force = self.max_cuff_force.max(tau_exo.abs() / CUFF_M);
        self.motor.step_thermal(DT, self.ambient_c);
        self.max_winding_c = self.max_winding_c.max(self.motor.winding_temp_c());
        if self.motor.current().abs() > 0.8 * self.cfg.i_max_a as f64
            && self.plant.omega.abs() < 0.01
        {
            self.stall_s += DT;
        } else {
            self.stall_s = 0.0;
        }
        // 5. Батарея: мощность моста + электроника.
        let i_bat = if vbat > 1.0 {
            (v_motor * self.motor.current()).max(-50.0 * vbat) / vbat + ELECTRONICS_A
        } else {
            ELECTRONICS_A
        };
        self.pack.step(i_bat, DT);
        // 6. Радио.
        let budget = self.link().budget();
        let per = if self.faults.radio_off {
            1.0
        } else {
            packet_error_rate(self.phy(), budget.margin_db(), 30.0 * 8.0)
        };
        self.channel.set_link(per, budget.rx_dbm());
        let mcu = self.mcu.as_mut().unwrap();
        for pkt in mcu.radio_take_tx() {
            self.channel.send(self.t, Dir::DeviceToPhone, &pkt);
        }
        if self.t >= self.next_cmd {
            self.seq = self.seq.wrapping_add(1);
            let cmd = ExoCommand {
                mode: self.exo_mode(),
                assist_pct: self.assist_pct.round() as u8,
                target_deg: self.target_deg,
                payload_kg: self.phone_payload,
                seq: self.seq,
            };
            self.channel.send(self.t, Dir::PhoneToDevice, &cmd.encode());
            self.next_cmd += CMD_PERIOD;
        }
        let deliveries = self.channel.advance(self.t);
        let mcu = self.mcu.as_mut().unwrap();
        for d in deliveries {
            match d.dir {
                Dir::PhoneToDevice => {
                    mcu.radio_deliver(&d.bytes, d.rssi_dbm.clamp(-127.0, 0.0) as i8);
                }
                Dir::DeviceToPhone => {
                    if let Some(tel) = ExoTelemetry::decode(&d.bytes) {
                        let old = self.last_tel.map(|t| t.all_faults()).unwrap_or(0);
                        if tel.all_faults() != old {
                            let names = fault::names(tel.all_faults() & !old);
                            if !names.is_empty() {
                                self.log.push(format!(
                                    "{:8.3} с  телефон: прошивка сообщает {}",
                                    self.t,
                                    names.join(", ")
                                ));
                            }
                        }
                        self.last_tel = Some(tel);
                        self.tel_count += 1;
                    }
                }
            }
        }
        if !uart.is_empty() {
            for line in uart.lines().filter(|l| !l.trim().is_empty()) {
                self.log.push(format!("{:8.3} с  UART: {line}", self.t));
            }
        }
        let reported_faults = self.last_faults();
        if let Some(trace) = &mut self.trace {
            trace.record(LoopSample {
                t: self.t,
                theta: self.plant.theta,
                omega: self.plant.omega,
                current_a: self.motor.current(),
                motor_v: v_motor,
                duty,
                joint_torque: tau_exo,
                human_torque: self.human_torque,
                stop_torque: self.plant.stop_torque,
                contact_n: force,
                battery_v: vbat,
                winding_c: self.motor.winding_temp_c(),
                housing_c: self.motor.housing_temp_c(),
                driver_on: enabled,
                reported_faults,
            });
        }
        self.t += DT;
        // 7. История (каждые 2 мс).
        if ((self.t / DT).round() as u64).is_multiple_of(HIST_EVERY) {
            let i_ref =
                self.cfg
                    .gravity_assist_current(self.assist_pct, self.phone_payload, theta_deg);
            let tel_angle = self.last_tel.map(|t| t.angle_deg).unwrap_or(f64::NAN);
            self.hist.push(
                self.t,
                &[
                    theta_deg,
                    self.target_deg,
                    self.motor.current(),
                    i_ref,
                    self.motor.winding_temp_c(),
                    self.pack.terminal_voltage(),
                    tau_exo,
                    self.human_torque,
                    tel_angle,
                ],
            );
        }
    }

    fn start_rebuild(&mut self) {
        let dir = firmware_dir();
        let slot: BuildSlot = Arc::new(Mutex::new(None));
        let tx = slot.clone();
        std::thread::spawn(move || {
            let out = std::process::Command::new("sh")
                .arg("build.sh")
                .current_dir(&dir)
                .output();
            let res = match out {
                Ok(o) if o.status.success() => Ok(String::from_utf8_lossy(&o.stdout).to_string()),
                Ok(o) => Err(String::from_utf8_lossy(&o.stderr).to_string()),
                Err(e) => Err(e.to_string()),
            };
            if let Ok(mut g) = tx.lock() {
                *g = Some(res);
            }
        });
        self.build_rx = Some(slot);
        self.log("пересборка прошивки запущена (firmware/build.sh)…".into());
    }

    fn poll_rebuild(&mut self) {
        let Some(slot) = &self.build_rx else {
            return;
        };
        let done = slot.lock().ok().and_then(|mut g| g.take());
        if let Some(res) = done {
            self.build_rx = None;
            match res {
                Ok(_) => {
                    self.log("прошивка пересобрана — загружаю в МК".into());
                    self.load_firmware(0);
                }
                Err(e) => {
                    let first = e
                        .lines()
                        .find(|l| l.contains("error"))
                        .unwrap_or("ошибка сборки");
                    let msg = format!("сборка не удалась: {first}");
                    self.log(msg);
                }
            }
        }
    }

    /// Средний ток мотора за последние `w` секунд.
    fn mean_current(&self, w: f64) -> Option<f64> {
        self.hist.mean(2, w)
    }

    /// Числа для тестов: (время, угол °, ток А, ток по ручному расчёту А, T обмотки,
    /// момент человека Н·м, сброшен ли сторожевым таймером, реакция на аварийную кнопку с).
    pub fn probe(&self) -> (f64, f64, f64, f64, f64, f64, u32, Option<f64>) {
        let th = self.plant.theta.to_degrees();
        (
            self.t,
            th,
            self.mean_current(0.3).unwrap_or(0.0),
            self.cfg
                .gravity_assist_current(self.assist_pct, self.phone_payload, th),
            self.motor.winding_temp_c(),
            self.human_torque,
            self.mcu.as_ref().map(|m| m.watchdog_resets()).unwrap_or(0),
            self.estop_reaction,
        )
    }

    /// Все флаги неисправностей из последней телеметрии (байт 1 + байт 2 << 8).
    /// Начать запись: редкий отсчёт раз в `every_steps` шагов, отпечаток по всем шагам.
    pub fn start_trace(&mut self, every_steps: u64) {
        self.trace = Some(LoopTrace::new(every_steps));
    }

    pub fn take_trace(&mut self) -> Option<LoopTrace> {
        self.trace.take()
    }

    pub fn last_faults(&self) -> u16 {
        self.last_tel.map(|t| t.all_faults()).unwrap_or(0)
    }

    /// Диагностика прошивки: оценка температуры обмотки и пр.
    pub fn diag(&self) -> Option<ExoDiag> {
        self.mcu.as_ref().and_then(ExoDiag::read)
    }

    pub fn winding_temp_c(&self) -> f64 {
        self.motor.winding_temp_c()
    }

    /// Поставить препятствие на пути сгибания (или убрать). Журнал контакта начинается заново.
    pub fn set_obstacle(&mut self, obstacle: Option<Obstacle>) {
        self.obstacle = obstacle;
        self.plant.obstacle = obstacle;
        self.contact = ContactLog::default();
    }

    pub fn contact_log(&self) -> ContactLog {
        self.contact
    }

    fn contact_limit(&self) -> Option<ContactLimit> {
        match self.obstacle_choice {
            1 => Some(CONTACT_FOREARM),
            2 => Some(CONTACT_HAND),
            _ => None,
        }
    }

    fn choose_obstacle(&mut self, index: usize) {
        self.obstacle_choice = index.min(2);
        let lever = if self.obstacle_choice == 1 { CUFF_M } else { self.plant.p.hand };
        let angle = self.plant.theta + 15f64.to_radians();
        let obstacle = self.contact_limit().map(|l| Obstacle {
            angle,
            lever,
            stiffness: l.stiffness_n_per_m,
            damping: 0.0,
        });
        self.set_obstacle(obstacle);
    }

    /// Конфигурация, которая реально записана в МК.
    pub fn firmware_config(&self) -> ExoConfig {
        self.cfg
    }

    pub fn driver_enabled(&self) -> bool {
        self.mcu
            .as_ref()
            .map(|m| m.gpio_output(pins::GPIO_ENABLE))
            .unwrap_or(false)
    }

    pub fn firmware_error(&self) -> Option<&str> {
        self.fw_error.as_deref()
    }
}

impl Scenario for ExoLoop {
    fn id(&self) -> &'static str {
        "exo_loop"
    }

    fn title(&self) -> String {
        "Экзо целиком: прошивка в контуре".into()
    }

    fn summary(&self) -> String {
        "Настоящая прошивка (машинный код RISC-V) на виртуальном микроконтроллере управляет \
         мотором с редуктором, батареей и радио; локоть — с инерцией из анатомической модели. \
         Включайте отказы кнопками и смотрите, как прошивка их ловит."
            .into()
    }

    fn params(&self) -> Vec<Param> {
        let p = |id, label: &str, value, min, max, unit, decimals| Param {
            id,
            label: label.to_string(),
            value,
            min,
            max,
            unit,
            decimals,
        };
        vec![
            p(
                "assist",
                "Телефон: поддержка",
                self.assist_pct,
                0.0,
                100.0,
                "%",
                0,
            ),
            p(
                "target",
                "Телефон: держать угол",
                self.target_deg,
                5.0,
                120.0,
                "°",
                0,
            ),
            p(
                "phone_payload",
                "Телефон: груз (введён)",
                self.phone_payload,
                0.0,
                30.0,
                "кг",
                1,
            ),
            p(
                "payload",
                "Груз в руке (на самом деле)",
                self.payload,
                0.0,
                30.0,
                "кг",
                1,
            ),
            p(
                "human_target",
                "Человек держит угол",
                self.human_target,
                10.0,
                120.0,
                "°",
                0,
            ),
            p(
                "ambient",
                "Температура воздуха",
                self.ambient_c,
                -30.0,
                50.0,
                "°C",
                0,
            ),
            p(
                "distance",
                "Телефон далеко",
                self.distance,
                0.5,
                30.0,
                "м",
                1,
            ),
        ]
    }

    fn set_param(&mut self, id: &str, value: f64) {
        match id {
            "assist" => self.assist_pct = value,
            "target" => self.target_deg = value,
            "phone_payload" => self.phone_payload = value,
            "payload" => {
                self.payload = value;
                self.plant.load.payload = value;
            }
            "human_target" => self.human_target = value,
            "ambient" => {
                self.ambient_c = value;
                self.pack.set_temperature(value);
            }
            "distance" => self.distance = value,
            _ => {}
        }
    }

    fn choices(&self) -> Vec<Choice> {
        vec![
            Choice {
                id: "mode",
                label: "Режим (с телефона)".into(),
                options: MODES.iter().map(|s| s.to_string()).collect(),
                selected: self.mode,
            },
            Choice {
                id: "human",
                label: "Человек".into(),
                options: HUMANS.iter().map(|s| s.to_string()).collect(),
                selected: self.human,
            },
            Choice {
                id: "motor",
                label: "Привод".into(),
                options: MOTORS.iter().map(|s| s.to_string()).collect(),
                selected: self.motor_choice,
            },
            Choice {
                id: "pack",
                label: "Батарея".into(),
                options: vec![
                    "6S2P Li-ion (≈22 В)".into(),
                    "12S2P Li-ion (≈44 В) с делителем от 6S".into(),
                ],
                selected: (self.pack_s == 12) as usize,
            },
            Choice {
                id: "medium",
                label: "Среда".into(),
                options: vec!["воздух".into(), "рука в воде".into()],
                selected: self.water as usize,
            },
            Choice {
                id: "radio",
                label: "Связь".into(),
                options: vec![
                    "Bluetooth LE (интервал 15 мс)".into(),
                    "Wi-Fi 802.11n".into(),
                ],
                selected: self.wifi as usize,
            },
            Choice {
                id: "body",
                label: "Телефон".into(),
                options: vec![
                    "перед человеком".into(),
                    "за спиной (тело закрывает)".into(),
                ],
                selected: self.body_blocks as usize,
            },
            Choice {
                id: "obstacle",
                label: "Препятствие на пути руки".into(),
                options: vec![
                    "нет".into(),
                    "упирается предплечье".into(),
                    "упирается кисть".into(),
                ],
                selected: self.obstacle_choice,
            },
        ]
    }

    fn set_choice(&mut self, id: &str, index: usize) {
        match id {
            "mode" => self.mode = index.min(2),
            "human" => {
                self.human = index.min(2);
                self.human_integ = 0.0;
            }
            "motor" => {
                self.motor_choice = index.min(1);
                self.reset();
            }
            "medium" => {
                self.water = index == 1;
                self.apply_medium();
            }
            "pack" => {
                self.pack_s = if index == 1 { 12 } else { 6 };
                self.reset();
            }
            "body" => self.body_blocks = index == 1,
            "obstacle" => self.choose_obstacle(index),
            "radio" => {
                self.wifi = index == 1;
                self.channel = PacketChannel::new(
                    if self.wifi {
                        Protocol::Wifi
                    } else {
                        Protocol::ble_default()
                    },
                    7,
                );
            }
            _ => {}
        }
    }

    fn presets(&self) -> Vec<String> {
        vec![
            "Помощь 70 %, груз 10 кг".into(),
            "Ввели не тот груз (0 вместо 10)".into(),
            "Держать 60°, человек расслаблен".into(),
            "Мороз −20 °C".into(),
            "Рука в воде".into(),
            "Рука упёрлась в препятствие".into(),
        ]
    }

    fn apply_preset(&mut self, index: usize) {
        // Каждый вариант начинается с чистых условий, а не с остатков предыдущего.
        self.mode = 1;
        self.human = 0;
        self.target_deg = 90.0;
        self.human_target = if index == 5 { 30.0 } else { 90.0 };
        self.phone_payload = self.payload;
        self.ambient_c = 22.0;
        self.water = false;
        self.obstacle = None;
        self.obstacle_choice = 0;
        self.reset();
        match index {
            0 => {}
            5 => {
                self.mode = 2;
                self.human = 1;
                self.target_deg = 110.0;
                self.choose_obstacle(1);
            }
            1 => self.phone_payload = 0.0,
            2 => {
                self.mode = 2;
                self.target_deg = 60.0;
                self.human = 1;
            }
            3 => {
                self.ambient_c = -20.0;
                self.pack.set_temperature(-20.0);
                self.motor = Motor::new(self.motor_params()).with_temperatures(-20.0, -20.0);
            }
            _ => {
                self.water = true;
                self.apply_medium();
            }
        }
        self.load_firmware(0);
    }

    fn actions(&self) -> Vec<Action> {
        vec![
            Action::new(
                "estop",
                "Аварийная кнопка",
                "Нажать/отпустить: вход GPIO0 → 0",
                self.faults.estop,
            ),
            Action::new(
                "sensor",
                "Обрыв датчика угла",
                "АЦП0 = 0 В (провод оборван)",
                self.faults.sensor_break,
            ),
            Action::new(
                "jam",
                "Заклинило сустав",
                "Сустав не движется, мотор упирается",
                self.faults.jam,
            ),
            Action::new(
                "radio",
                "Пропала связь",
                "Ни один пакет не проходит",
                self.faults.radio_off,
            ),
            Action::new(
                "hang",
                "Зависание прошивки",
                "Основной цикл виснет через 100 мс — сработает ли сторожевой таймер",
                false,
            ),
            Action::new(
                "reset_mcu",
                "Сброс МК",
                "Перезапуск прошивки с обычной конфигурацией",
                false,
            ),
            Action::new(
                "auto_cfg",
                "Конфигурация по расчёту",
                "Отбросить ручные правки: усиления и пределы — расчёт под мотор, батарею и руку",
                self.user_cfg.is_some(),
            ),
            Action::new(
                "rebuild",
                if self.build_rx.is_some() {
                    "Сборка идёт…"
                } else {
                    "Пересобрать прошивку"
                },
                "cargo build прошивки firmware/exo-ctrl под RISC-V и загрузка в МК",
                self.build_rx.is_some(),
            ),
        ]
    }

    fn run_action(&mut self, id: &str) {
        match id {
            "estop" => {
                self.faults.estop = !self.faults.estop;
                if self.faults.estop {
                    self.estop_at = Some(self.t);
                    self.estop_reaction = None;
                    self.log("нажата аварийная кнопка".into());
                } else {
                    self.log("аварийная кнопка отпущена (сброс — командой «выключено»)".into());
                }
            }
            "sensor" => {
                self.faults.sensor_break = !self.faults.sensor_break;
                let s = if self.faults.sensor_break {
                    "оборван"
                } else {
                    "восстановлен"
                };
                self.log(format!("датчик угла {s}"));
            }
            "jam" => {
                self.faults.jam = !self.faults.jam;
                self.plant.jammed = self.faults.jam;
                let s = if self.faults.jam {
                    "заклинило"
                } else {
                    "освобождён"
                };
                self.log(format!("сустав {s}"));
            }
            "radio" => {
                self.faults.radio_off = !self.faults.radio_off;
                let s = if self.faults.radio_off {
                    "пропала"
                } else {
                    "вернулась"
                };
                self.log(format!("связь {s}"));
            }
            "hang" => {
                self.log("загружаю прошивку с имитацией зависания через 100 мс".into());
                self.load_firmware(100);
            }
            "reset_mcu" => {
                self.log("сброс МК".into());
                self.load_firmware(0);
            }
            "auto_cfg" => {
                self.user_cfg = None;
                self.log("конфигурация — по расчёту под мотор и руку".into());
                self.load_firmware(0);
            }
            "rebuild" if self.build_rx.is_none() => self.start_rebuild(),
            _ => {}
        }
    }

    fn reset(&mut self) {
        let params = self.plant.p.clone();
        self.plant = ElbowPlant::new(params, self.human_target.to_radians());
        self.plant.load.payload = self.payload;
        // Препятствие всегда впереди руки: иначе старт начался бы с огромного вдавливания.
        if let Some(o) = &mut self.obstacle
            && o.angle <= self.plant.theta
        {
            o.angle = self.plant.theta + 15f64.to_radians();
        }
        self.plant.obstacle = self.obstacle;
        self.contact = ContactLog::default();
        self.motor =
            Motor::new(self.motor_params()).with_temperatures(self.ambient_c, self.ambient_c);
        self.pack = Pack::new(self.pack_s, 2, self.ambient_c);
        self.channel = PacketChannel::new(
            if self.wifi {
                Protocol::Wifi
            } else {
                Protocol::ble_default()
            },
            7,
        );
        self.apply_medium();
        self.faults = Faults::default();
        self.t = 0.0;
        self.clock.reset();
        self.hist.clear();
        self.log.clear();
        self.next_cmd = 0.0;
        self.last_tel = None;
        self.tel_count = 0;
        self.human_integ = 0.0;
        self.estop_at = None;
        self.estop_reaction = None;
        self.stop_hits = 0;
        self.max_cuff_force = 0.0;
        self.stall_s = 0.0;
        self.max_winding_c = self.ambient_c;
        self.load_firmware(0);
        self.epoch += 1;
        self.log(format!(
            "старт: {}, батарея {}S2P, прошивка {}",
            MOTORS[self.motor_choice],
            self.pack_s,
            exo_ctrl_elf()
                .file_name()
                .and_then(|s| s.to_str())
                .unwrap_or("?")
        ));
    }

    fn advance(&mut self, dt_real: f64) {
        self.poll_rebuild();
        for _ in 0..self.clock.steps(dt_real) {
            self.step();
        }
        if self.log.len() > 300 {
            self.log.drain(..self.log.len() - 300);
        }
    }

    fn time(&self) -> f64 {
        self.t
    }

    fn visuals(&self) -> Vec<VisualInstance> {
        let elbow = Vec3::new(0.0, 0.0, 1.1);
        let th = self.plant.theta;
        // Плечо вертикально вверх от локтя, предплечье: θ = 0 — вниз, сгибание — вперёд (+X).
        let fore = Vec3::new(th.sin(), 0.0, -th.cos());
        let hand = elbow + fore * self.plant.p.hand;
        let mut v = vec![
            VisualInstance {
                shape: Shape::Capsule {
                    radius: 0.045,
                    length: 0.28,
                },
                pose: Pose::z_along(elbow + Vec3::z() * 0.16, Vec3::z()),
                color: C_SKIN,
            },
            VisualInstance {
                shape: Shape::Capsule {
                    radius: 0.037,
                    length: self.plant.p.hand - 0.05,
                },
                pose: Pose::z_along(elbow + fore * (self.plant.p.hand / 2.0), fore),
                color: C_SKIN,
            },
            // Привод на оси локтя (снаружи).
            VisualInstance {
                shape: Shape::Cylinder {
                    radius: 0.05,
                    length: 0.05,
                },
                pose: Pose::z_along(elbow - Vec3::y() * 0.07, Vec3::y()),
                color: if self.driver_enabled() {
                    C_MOTOR
                } else {
                    C_GRAY
                },
            },
            // Штанги ортеза.
            VisualInstance {
                shape: Shape::Box {
                    half: Vec3::new(0.012, 0.006, 0.13),
                },
                pose: Pose::at(elbow - Vec3::y() * 0.07 + Vec3::z() * 0.14),
                color: C_EXO,
            },
            VisualInstance {
                shape: Shape::Capsule {
                    radius: 0.01,
                    length: CUFF_M + 0.05,
                },
                pose: Pose::z_along(
                    elbow - Vec3::y() * 0.07 + fore * ((CUFF_M + 0.05) / 2.0),
                    fore,
                ),
                color: C_EXO,
            },
            // Манжета.
            VisualInstance {
                shape: Shape::Cylinder {
                    radius: 0.05,
                    length: 0.06,
                },
                pose: Pose::z_along(elbow + fore * CUFF_M, fore),
                color: C_EXO_CUFF,
            },
            // Плата с МК на плече и светодиод.
            VisualInstance {
                shape: Shape::Box {
                    half: Vec3::new(0.025, 0.002, 0.035),
                },
                pose: Pose::at(elbow - Vec3::y() * 0.05 + Vec3::z() * 0.22),
                color: [0.1, 0.45, 0.2, 1.0],
            },
            VisualInstance {
                shape: Shape::Sphere { radius: 0.006 },
                pose: Pose::at(elbow - Vec3::y() * 0.054 + Vec3::z() * 0.25),
                color: if self
                    .mcu
                    .as_ref()
                    .map(|m| m.gpio_output(pins::GPIO_LED))
                    .unwrap_or(false)
                {
                    C_GREEN
                } else {
                    [0.1, 0.15, 0.1, 1.0]
                },
            },
            // Батарея на поясе.
            VisualInstance {
                shape: Shape::Box {
                    half: Vec3::new(0.05, 0.035, 0.08),
                },
                pose: Pose::at(Vec3::new(-0.08, 0.0, 0.85)),
                color: [0.15, 0.2, 0.35, 1.0],
            },
        ];
        if self.payload > 0.0 {
            let r = (3.0 * self.payload / (4.0 * std::f64::consts::PI * 7200.0)).cbrt();
            v.push(VisualInstance {
                shape: Shape::Sphere { radius: r },
                pose: Pose::at(hand - Vec3::z() * (r + 0.04)),
                color: C_IRON,
            });
        }
        if self.water {
            v.push(VisualInstance {
                shape: Shape::Box {
                    half: Vec3::new(0.6, 0.6, 0.6),
                },
                pose: Pose::at(Vec3::new(0.1, 0.0, 0.6)),
                color: [0.25, 0.55, 0.9, 0.22],
            });
        }
        // Препятствие: плита поперёк пути руки, красная, пока на неё давят.
        if let Some(o) = self.obstacle {
            let along = Vec3::new(o.angle.sin(), 0.0, -o.angle.cos());
            let tangent = Vec3::new(o.angle.cos(), 0.0, o.angle.sin());
            v.push(VisualInstance {
                shape: Shape::Box {
                    half: Vec3::new(0.09, 0.09, 0.012),
                },
                pose: Pose::z_along(elbow + along * o.lever + tangent * 0.05, tangent),
                color: if self.plant.contact_force > 0.0 { C_RED } else { C_GRAY },
            });
        }
        // Телефон.
        let dir = if self.body_blocks { -1.0 } else { 1.0 };
        v.push(VisualInstance {
            shape: Shape::Box {
                half: Vec3::new(0.004, 0.036, 0.075),
            },
            pose: Pose::at(Vec3::new(dir * self.distance.min(4.0), 0.3, 1.0)),
            color: [0.1, 0.1, 0.12, 1.0],
        });
        v
    }

    fn visual_epoch(&self) -> u64 {
        self.epoch * 1000
            + self.water as u64 * 7
            + (self.payload * 10.0) as u64 * 13
            + self.body_blocks as u64
            + (self.distance * 10.0) as u64 * 101
    }

    fn overlays(&self) -> Vec<Overlay> {
        let elbow = Vec3::new(0.0, 0.0, 1.1);
        let th = self.plant.theta;
        let fore = Vec3::new(th.sin(), 0.0, -th.cos());
        let cuff = elbow + fore * CUFF_M;
        let tau = self.motor.joint_torque();
        // Сила манжеты на предплечье: перпендикулярно предплечью в плоскости сгибания.
        let n = Vec3::new(th.cos(), 0.0, th.sin()) * tau.signum();
        let f = tau.abs() / CUFF_M;
        let mut v = vec![
            Overlay::Arrow {
                from: cuff,
                to: cuff + n * (0.0015 * f).min(0.4),
                color: C_YELLOW,
            },
            Overlay::Ring {
                center: elbow - Vec3::y() * 0.07,
                normal: Vec3::y(),
                radius: 0.06 + 0.002 * tau.abs().min(80.0),
                color: C_MOTOR,
            },
        ];
        let dir = if self.body_blocks { -1.0 } else { 1.0 };
        let phone = Vec3::new(dir * self.distance.min(4.0), 0.3, 1.0);
        let ok = self.channel.connected() && !self.faults.radio_off;
        v.push(Overlay::Line {
            from: elbow - Vec3::y() * 0.05 + Vec3::z() * 0.22,
            to: phone,
            color: if ok {
                [0.3, 0.8, 1.0, 0.6]
            } else {
                [1.0, 0.25, 0.2, 0.6]
            },
        });
        v
    }

    fn readouts(&self) -> Vec<Readout> {
        let th = self.plant.theta.to_degrees();
        let tau = self.motor.joint_torque();
        let hold = self.plant.holding_torque(self.plant.theta);
        let mut r = vec![
            Readout::new("Угол локтя", th, "°", 1),
            Readout::new("Нужно держать (груз + рука)", hold, "Н·м", 1),
            Readout::new("Даёт экзо", tau, "Н·м", 1),
            Readout::new("Даёт человек (мышцы)", self.human_torque, "Н·м", 1),
            Readout::new("Ток мотора", self.motor.current(), "А", 2),
            Readout::new("Сила манжеты на предплечье", tau.abs() / CUFF_M, "Н", 0),
            Readout::new("Обмотка мотора", self.motor.winding_temp_c(), "°C", 1),
            Readout::new("Корпус мотора", self.motor.housing_temp_c(), "°C", 1),
            Readout::new("Сила в препятствии", self.plant.contact_force, "Н", 0),
            Readout::new("Батарея: напряжение", self.pack.terminal_voltage(), "В", 2),
            Readout::new("Батарея: заряд", 100.0 * self.pack.soc(), "%", 1),
            Readout::new(
                "Батарея: доступно при этой температуре",
                100.0 * self.pack.capacity_fraction(),
                "%",
                0,
            ),
        ];
        if let Some(m) = &self.mcu {
            r.push(Readout::new(
                "МК: загрузка процессора",
                100.0 * m.cpu_load(),
                "%",
                1,
            ));
            r.push(Readout::new(
                "МК: инструкций выполнено",
                m.instret() as f64,
                "",
                0,
            ));
            r.push(Readout::new(
                "МК: сбросов сторожевым таймером",
                m.watchdog_resets() as f64,
                "",
                0,
            ));
        }
        let st = &self.channel.stats;
        r.push(Readout::new(
            "Радио: пакетов телеметрии принято",
            self.tel_count as f64,
            "",
            0,
        ));
        r.push(Readout::new(
            "Радио: задержка (средняя)",
            1000.0 * st.mean_latency_s(),
            "мс",
            1,
        ));
        r.push(Readout::new(
            "Радио: потеряно",
            100.0 * st.loss_fraction(),
            "%",
            1,
        ));
        if let Some(t) = &self.last_tel {
            r.push(Readout::new("Телефон видит угол", t.angle_deg, "°", 1));
            r.push(Readout::new("Телефон видит ток", t.current_a, "А", 2));
        }
        r
    }

    fn comparisons(&self) -> Vec<Comparison> {
        let mut v = Vec::new();
        let th = self.plant.theta.to_degrees();
        // 0. Контакт с препятствием против пределов ISO/TS 15066.
        if let (Some(l), Some(t0)) = (self.contact_limit(), self.contact.first_touch_s) {
            let hit_limit = TRANSIENT_FACTOR * l.quasi_static_n;
            v.push(Comparison {
                title: format!("Удар: {}", l.name),
                sim_label: "пик силы, прошивка в контуре".into(),
                sim: self.contact.peak_n,
                ref_label: "предел удара (первые 0.5 с)".into(),
                reference: hit_limit,
                unit: "Н",
                decimals: 0,
                status: if self.contact.peak_n <= hit_limit { Status::WithinLimit } else { Status::OverLimit },
                explanation: format!(
                    "Скорость касания {:.2} м/с. Инерция привода через редуктор бьёт вместе с рукой. Предел ISO/TS 15066 написан для роботов и близок к началу боли.",
                    self.contact.impact_speed_m_s
                ),
            });
            let late = self.t - t0 > TRANSIENT_WINDOW_S;
            v.push(Comparison {
                title: format!("Зажим: {}", l.name),
                sim_label: "сила после 0.5 с контакта".into(),
                sim: self.contact.peak_late_n,
                ref_label: "предел зажима".into(),
                reference: l.quasi_static_n,
                unit: "Н",
                decimals: 0,
                status: if !late {
                    Status::Pending
                } else if self.contact.peak_late_n <= l.quasi_static_n {
                    Status::WithinLimit
                } else {
                    Status::OverLimit
                },
                explanation: "Привод продолжает давить, пока не сработает защита от заклинивания.".into(),
            });
        }
        // 1. Ток поддержки: прошивка против ручного расчёта.
        if self.mode == 1 {
            let measured = self.mean_current(0.3).unwrap_or(f64::NAN);
            let hand = self
                .cfg
                .gravity_assist_current(self.assist_pct, self.phone_payload, th);
            let still = self.plant.omega.abs() < 0.05 && self.t > 1.0;
            v.push(Comparison {
                title: "Ток поддержки: прошивка ↔ ручной расчёт".into(),
                sim_label: "прошивка в контуре (среднее 0.3 с)".into(),
                sim: measured,
                ref_label: "τ/(Kt·N·η), τ = доля·(m·g·r + M·g·L)·sin θ".into(),
                reference: hand,
                unit: "А",
                decimals: 3,
                status: if !still || self.fw_error.is_some() || self.last_faults() != 0 {
                    Status::Pending
                } else if (measured - hand).abs() < 0.05 + 0.03 * hand.abs() {
                    Status::Ok
                } else {
                    Status::Mismatch
                },
                explanation: "Второй способ — та же формула вручную по конфигурации прошивки. \
                              Ждём, пока рука остановится и пройдут переходные процессы."
                    .into(),
            });
        }
        // 2. Груз, введённый в телефон, против настоящего.
        if self.mode == 1 && (self.phone_payload - self.payload).abs() > 0.5 {
            let hold = self.plant.holding_torque(self.plant.theta);
            v.push(Comparison {
                title: "Экзо считает груз по телефону, а не по факту".into(),
                sim_label: "даёт экзо".into(),
                sim: self.motor.joint_torque(),
                ref_label: "доля от нужного на самом деле".into(),
                reference: self.assist_pct / 100.0 * hold,
                unit: "Н·м",
                decimals: 1,
                status: Status::Explained,
                explanation: "Помощь «по модели веса» верна, только пока груз введён правильно. \
                              Нужен датчик силы/момента в манжете или оценка груза по току."
                    .into(),
            });
        }
        // 2б. Трение редуктора в покое.
        let p = self.motor_params();
        let tpa = p.torque_constant_nm_per_a * p.gear.ratio;
        let i = self.motor.current();
        if self.plant.omega.abs() < 0.02 && i.abs() > 0.1 && self.t > 1.0 {
            v.push(Comparison {
                title: "Почему человеку почти не нужно держать: трение редуктора".into(),
                sim_label: "экзо даёт сейчас".into(),
                sim: self.motor.joint_torque(),
                ref_label: "Kt·N·η·i (по току, как считает прошивка)".into(),
                reference: tpa * p.gear.efficiency_forward * i,
                unit: "Н·м",
                decimals: 1,
                status: Status::Explained,
                explanation: format!(
                    "В покое редуктор с КПД обратного хода {:.2} держит трением до Kt·N·i/η_обр \
                     = {:.1} Н·м: часть веса держит не мотор и не человек. При подъёме руки \
                     трение будет мешать (вниз — помогать). КПД — допущение модели.",
                    p.gear.efficiency_backward,
                    tpa * i / p.gear.efficiency_backward
                ),
            });
        }
        // 3. Отражённая инерция.
        let refl = self.motor.reflected_inertia();
        v.push(Comparison {
            title: "Инерция привода через редуктор против предплечья".into(),
            sim_label: "ротор × N²".into(),
            sim: refl,
            ref_label: "предплечье с кистью".into(),
            reference: self.plant.p.inertia,
            unit: "кг·м²",
            decimals: 3,
            status: Status::Explained,
            explanation: format!(
                "Привод в {:.0} раз «тяжелее» руки при разгоне: без тока экзо почти не \
                 отпускает руку (не возвратный), рывки человек чувствует сильнее.",
                refl / self.plant.p.inertia
            ),
        });
        // 4. Аварийная кнопка.
        if let Some(r) = self.estop_reaction {
            v.push(Comparison {
                title: "Аварийная кнопка: время до отключения драйвера".into(),
                sim_label: "прошивка в контуре".into(),
                sim: 1000.0 * r,
                ref_label: "требование: ≤ 1 тик 10 кГц (+10 мкс)".into(),
                reference: 0.1,
                unit: "мс",
                decimals: 2,
                status: if r <= 110e-6 {
                    Status::WithinLimit
                } else {
                    Status::OverLimit
                },
                explanation: "Меряем от нажатия до снятия разрешения драйвера (GPIO2).".into(),
            });
        }
        // 5. Нагрев обмотки: куда придёт при этом токе.
        let p = self.motor_params();
        let i_rms = self.hist.mean(2, 5.0).unwrap_or(0.0).abs();
        if i_rms > 0.05 {
            let rth = p.thermal.r_winding_housing_k_per_w + self.motor.housing_to_ambient_k_per_w();
            // T = Ta + I²·R(T)·Rth, R(T) = R₂₅(1 + α(T − 25)) → решаем линейное уравнение.
            let alpha = 0.00393;
            let r25 = p.resistance_ohm * (1.0 + alpha * (25.0 - p.resistance_ref_c));
            let k = i_rms * i_rms * r25 * rth;
            let t_ss = (self.ambient_c + k * (1.0 - 25.0 * alpha)) / (1.0 - k * alpha);
            v.push(Comparison {
                title: "Обмотка: где установится температура".into(),
                sim_label: "сейчас (модель, растёт минутами)".into(),
                sim: self.motor.winding_temp_c(),
                ref_label: "стационар I²R(T)·(R₁+R₂)".into(),
                reference: t_ss,
                unit: "°C",
                decimals: 1,
                status: Status::Pending,
                explanation: format!(
                    "Тепловые постоянные {:.0} с и {:.0} с: равновесие — через ~{:.0} мин. Предел \
                     обмотки {:.0} °C.",
                    p.thermal.tau_winding_s,
                    p.thermal.tau_housing_s,
                    4.0 * p.thermal.tau_housing_s / 60.0,
                    p.thermal.max_winding_temp_c
                ),
            });
        }
        // 5б. Оценка обмотки в прошивке против тепловой модели мотора.
        if let Some(d) = self.diag() {
            let truth = self.motor.winding_temp_c();
            v.push(Comparison {
                title: "Температура обмотки: оценка прошивки ↔ модель мотора".into(),
                sim_label: "I²t-модель в прошивке (по току и корпусу)".into(),
                sim: d.winding_est_c,
                ref_label: "двухузловая модель sim-em (истина стенда)".into(),
                reference: truth,
                unit: "°C",
                decimals: 2,
                status: if (d.winding_est_c - truth).abs() < 2.0 {
                    Status::Ok
                } else {
                    Status::Mismatch
                },
                explanation: format!(
                    "Датчик стоит на корпусе ({:.1} °C), обмотку прошивка считает сама: защита \
                     срабатывает раньше, чем корпус успеет нагреться. Предел тока сейчас {:.2} А.",
                    d.housing_c, d.i_limit_a
                ),
            });
        }
        // 6. Задержка радио.
        let st = &self.channel.stats;
        if st.delivered > 100
            && let Protocol::Ble { interval_s, .. } = self.channel.protocol
        {
            let per = packet_error_rate(self.phy(), self.link().budget().margin_db(), 240.0);
            let refl = sim_radio::channel::ble_mean_latency_analytic(interval_s, per.min(0.9));
            v.push(Comparison {
                title: "Задержка команд по Bluetooth".into(),
                sim_label: "канал в контуре".into(),
                sim: 1000.0 * st.mean_latency_s(),
                ref_label: "T/2 + T·p/(1 − p)".into(),
                reference: 1000.0 * refl,
                unit: "мс",
                decimals: 1,
                status: if (st.mean_latency_s() - refl).abs() / refl < 0.3 {
                    Status::Ok
                } else {
                    Status::Explained
                },
                explanation: "Команды раз в 50 мс, телеметрия раз в 20 мс — очередь удлиняет \
                              ожидание."
                    .into(),
            });
        }
        v
    }

    fn notes(&self) -> Vec<Note> {
        let mut n = Vec::new();
        if let Some(e) = &self.fw_error {
            n.push(Note::new(Level::Danger, format!("Прошивка: {e}")));
        }
        if let Some(l) = self.contact_limit()
            && (self.contact.peak_n > TRANSIENT_FACTOR * l.quasi_static_n
                || self.contact.peak_late_n > l.quasi_static_n)
        {
            n.push(Note::new(
                Level::Danger,
                format!(
                    "Привод давит на руку сильнее допустимого ({}): пик {:.0} Н, зажим {:.0} Н при пределах {:.0} и {:.0} Н.",
                    l.name,
                    self.contact.peak_n,
                    self.contact.peak_late_n,
                    TRANSIENT_FACTOR * l.quasi_static_n,
                    l.quasi_static_n
                ),
            ));
        }
        let flags = self.last_faults();
        if flags != 0 {
            n.push(Note::new(
                Level::Danger,
                format!(
                    "Прошивка сообщает: {}. Драйвер {}.",
                    fault::names(flags).join(", "),
                    if self.driver_enabled() {
                        "включён"
                    } else {
                        "выключен"
                    }
                ),
            ));
        }
        if let Some(m) = &self.mcu
            && m.watchdog_resets() > 0
        {
            n.push(Note::new(
                Level::Warn,
                format!(
                    "Сторожевой таймер перезапустил МК {} раз(а): зависшая прошивка не \
                     оставит мотор включённым.",
                    m.watchdog_resets()
                ),
            ));
        }
        let p = self.motor_params();
        if self.stall_s > 2.0 {
            n.push(Note::new(
                Level::Danger,
                format!(
                    "Сустав стоит, а прошивка {:.0} с держит {:.1} А: защиты от заклинивания нет. \
                     Датчик температуры — на корпусе (сейчас {:.0} °C), а греется обмотка ({:.0} \
                     °C): при заклинивании обмотка проходит предел {:.0} °C за ~1–1.5 мин, а \
                     снижение тока по корпусу начнётся только через ~3 мин. Нужно: отключать при \
                     «ток на пределе без движения» дольше 1–2 с и считать нагрев обмотки по току \
                     (I²t-модель) в прошивке.",
                    self.stall_s,
                    self.motor.current(),
                    self.motor.housing_temp_c(),
                    self.motor.winding_temp_c(),
                    p.thermal.max_winding_temp_c
                ),
            ));
        }
        if self.max_winding_c > p.thermal.max_winding_temp_c {
            n.push(Note::new(
                Level::Danger,
                format!(
                    "Обмотка нагревалась до {:.0} °C — выше предела {:.0} °C: изоляция повреждена, \
                     мотор под замену.",
                    self.max_winding_c, p.thermal.max_winding_temp_c
                ),
            ));
        }
        if self.motor.winding_temp_c() > p.thermal.max_winding_temp_c - 20.0 {
            n.push(Note::new(
                Level::Danger,
                format!(
                    "Обмотка {:.0} °C — близко к пределу {:.0} °C.",
                    self.motor.winding_temp_c(),
                    p.thermal.max_winding_temp_c
                ),
            ));
        }
        if self.motor.housing_temp_c() > 43.0 {
            n.push(Note::new(
                Level::Warn,
                format!(
                    "Корпус мотора {:.0} °C: при долгом касании кожи выше ~43 °C — ожог. Нужен \
                     кожух/зазор от руки.",
                    self.motor.housing_temp_c()
                ),
            ));
        }
        let f = self.motor.joint_torque().abs() / CUFF_M;
        if f > 160.0 {
            n.push(Note::new(
                Level::Warn,
                format!(
                    "Манжета давит на предплечье {f:.0} Н — больше 160 Н (ISO/TS 15066, ориентир)."
                ),
            ));
        }
        if self.ambient_c < 0.0 {
            n.push(Note::new(
                Level::Warn,
                format!(
                    "Холод {:.0} °C: батарея отдаёт ~{:.0} % ёмкости, сопротивление выросло — \
                     напряжение проседает под током. Заряжать Li-ion ниже 0 °C нельзя.",
                    self.ambient_c,
                    100.0 * self.pack.capacity_fraction()
                ),
            ));
        }
        if self.pack.terminal_voltage() / 11.0 > 3.3 {
            n.push(Note::new(
                Level::Danger,
                format!(
                    "Ошибка платы: делитель батареи 1:11 рассчитан на 6S (до 25 В), а батарея \
                     {:.0} В даёт на входе АЦП {:.1} В > 3.3 В. АЦП упирается в потолок, прошивка \
                     видит ~36 В и считает батарею севшей. Нужен делитель ~1:21 и новая шкала в \
                     прошивке.",
                    self.pack.terminal_voltage(),
                    self.pack.terminal_voltage() / 11.0
                ),
            ));
        }
        if self.motor_choice == 0 && self.pack_s == 6 {
            n.push(Note::new(
                Level::Info,
                "Мотор на 48 В от батареи 6S (22 В): момент тот же, наибольшая скорость сустава \
                 вдвое ниже (~100 °/с) — для поддержки хватает, для быстрых движений нет.",
            ));
        }
        if self.pack.terminal_voltage() < 3.3 * self.pack.series as f64 {
            n.push(Note::new(
                Level::Warn,
                "Батарея почти разряжена (< 3.3 В на элемент).",
            ));
        }
        if self.water {
            n.push(Note::new(
                Level::Warn,
                "Рука в воде: Архимед снимает ~88 % веса руки — помощь «по модели веса» \
                 перебарщивает; Bluetooth под водой почти не проходит (связь рвётся, прошивка \
                 через 0.5 с снимает усилие). Корпус должен быть герметичным (IP68).",
            ));
        }
        if self.stop_hits > 0 {
            n.push(Note::new(
                Level::Warn,
                "Сустав упирался в анатомический предел — проверьте мягкие упоры ортеза.",
            ));
        }
        n.push(Note::new(
            Level::Info,
            "Прошивка — настоящий машинный код RISC-V (RV32IMC, как в ESP32-C3), собранный из \
             firmware/exo-ctrl. Периферия МК — своя упрощённая (не регистры ESP32-C3): для \
             настоящей платы нужен слой драйверов, логика управления та же.",
        ));
        n
    }

    fn plots(&self) -> Vec<Plot> {
        let w = 15.0;
        vec![
            Plot {
                title: "Угол локтя".into(),
                x_label: "время, с".into(),
                y_label: "°".into(),
                series: vec![
                    Series {
                        name: "на самом деле".into(),
                        points: self.hist.points(0, w, 600),
                        color: C_YELLOW,
                        style: SeriesStyle::Solid,
                    },
                    Series {
                        name: "видит телефон".into(),
                        points: self.hist.points(8, w, 600),
                        color: C_BLUE,
                        style: SeriesStyle::Dashed,
                    },
                ],
                bands: vec![],
            },
            Plot {
                title: "Моменты в локте".into(),
                x_label: "время, с".into(),
                y_label: "Н·м".into(),
                series: vec![
                    Series {
                        name: "экзо".into(),
                        points: self.hist.points(6, w, 600),
                        color: C_MOTOR,
                        style: SeriesStyle::Solid,
                    },
                    Series {
                        name: "человек".into(),
                        points: self.hist.points(7, w, 600),
                        color: C_SKIN,
                        style: SeriesStyle::Solid,
                    },
                ],
                bands: vec![],
            },
            Plot {
                title: "Ток мотора".into(),
                x_label: "время, с".into(),
                y_label: "А".into(),
                series: vec![
                    Series {
                        name: "ток".into(),
                        points: self.hist.points(2, w, 600),
                        color: C_RED,
                        style: SeriesStyle::Solid,
                    },
                    Series {
                        name: "ручной расчёт поддержки".into(),
                        points: self.hist.points(3, w, 200),
                        color: C_GRAY,
                        style: SeriesStyle::Dashed,
                    },
                ],
                bands: vec![],
            },
        ]
    }

    fn console(&self) -> Vec<String> {
        self.log.clone()
    }

    fn text_fields(&self) -> Vec<TextField> {
        vec![TextField {
            id: "config",
            label: "Конфигурация прошивки (записывается в МК по «Применить», МК перезапускается)"
                .into(),
            text: config_to_text(&ExoConfig {
                test_hang_ms: 0,
                ..self.cfg
            }),
            rows: 12,
        }]
    }

    fn set_text(&mut self, id: &str, text: String) {
        if id != "config" {
            return;
        }
        match config_from_text(&self.cfg, &text) {
            Ok(c) => {
                self.user_cfg = Some(c);
                self.log("конфигурация из редактора записана в МК, перезапуск".into());
                self.load_firmware(0);
            }
            Err(e) => self.log(format!("конфигурация не принята: {e}")),
        }
    }

    fn focus(&self) -> Vec3 {
        Vec3::new(0.15, 0.0, 1.05)
    }

    fn view_offset(&self) -> Vec3 {
        Vec3::new(0.4, -1.4, 0.3)
    }
}

impl ExoLoop {
    fn apply_medium(&mut self) {
        if self.water {
            self.plant.load = ElbowLoad {
                payload: self.payload,
                arm_buoyancy: 0.88,
                payload_buoyancy: 1000.0 / 7200.0,
                // По MuJoCo: −2.27 Н·м при 3 рад/с → k ≈ 0.25 Н·м·с²/рад² (fluid.rs).
                drag_k: 0.25,
            };
            // Корпус в воде остывает в десятки раз лучше (h воды ~ сотни Вт/м²К против ~10).
            let r = self.motor_params().thermal.r_housing_ambient_k_per_w * 0.05;
            self.motor.set_housing_to_ambient_k_per_w(r);
        } else {
            self.plant.load = ElbowLoad {
                payload: self.payload,
                ..Default::default()
            };
            let r = self.motor_params().thermal.r_housing_ambient_k_per_w;
            self.motor.set_housing_to_ambient_k_per_w(r);
        }
    }
}
