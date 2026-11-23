//! Контракт прошивки `exo-ctrl` (контроллер локтевого экзоскелета) со стороны хоста.
//!
//! Приложение подключает физику ровно так (см. также `MEMORY_MAP.md`, «exo-ctrl»):
//!
//! | вход/выход | шкала |
//! |---|---|
//! | ADC0 — угол локтя θ (0° — рука прямая, + сгибание) | V = 0.3 + 2.7·θ/150; V < 0.15 или > 3.15 → обрыв датчика |
//! | ADC1 — ток мотора I (+ = момент сгибания) | V = 1.65 + 0.1·I |
//! | ADC2 — температура КОРПУСА двигателя/драйвера T | V = 0.5 + 0.01·T (как TMP36); обмотку прошивка оценивает тепловой моделью |
//! | ADC3 — батарея | V = Vbat/11 |
//! | GPIO-вход 0 — аварийная кнопка | активный низкий (по умолчанию высокий) |
//! | PWM0 — модуль заполнения H-моста 0..1 | |
//! | GPIO-выход 1 — направление (1 = момент сгибания), 2 — разрешение драйвера, 3 — индикатор | |
//!
//! Блок конфигурации — [`ExoConfig::to_bytes`], радиопротокол — [`ExoCommand`] и
//! [`ExoTelemetry`], флаги неисправностей — [`fault`].

use crate::Mcu;

/// Номера каналов и выводов.
pub mod pins {
    pub const ADC_ANGLE: usize = 0;
    pub const ADC_CURRENT: usize = 1;
    pub const ADC_TEMP: usize = 2;
    pub const ADC_VBAT: usize = 3;
    pub const GPIO_ESTOP: usize = 0;
    pub const GPIO_DIR: usize = 1;
    pub const GPIO_ENABLE: usize = 2;
    pub const GPIO_LED: usize = 3;
    pub const PWM_MOTOR: usize = 0;
}

/// Шкалы датчиков (функции «величина → вольты на входе АЦП» и обратные).
pub mod sensors {
    /// Граница исправности датчика угла, В.
    pub const ANGLE_FAULT_LOW_V: f64 = 0.15;
    pub const ANGLE_FAULT_HIGH_V: f64 = 3.15;

    pub fn angle_volts(theta_deg: f64) -> f64 {
        0.3 + 2.7 * theta_deg / 150.0
    }
    pub fn angle_from_volts(v: f64) -> f64 {
        (v - 0.3) * 150.0 / 2.7
    }
    pub fn current_volts(i_a: f64) -> f64 {
        1.65 + 0.1 * i_a
    }
    pub fn current_from_volts(v: f64) -> f64 {
        (v - 1.65) / 0.1
    }
    pub fn temp_volts(t_c: f64) -> f64 {
        0.5 + 0.01 * t_c
    }
    pub fn battery_volts(vbat: f64) -> f64 {
        vbat / 11.0
    }
}

/// Биты `fault_flags` (байт 10) и `fault_flags2` (байт 13) телеметрии. В виде u16:
/// младший байт — `fault_flags`, старший — `fault_flags2` ([`ExoTelemetry::all_faults`]).
pub mod fault {
    /// |I| > i_trip дольше 2 мс → драйвер выключен (защёлка до команды режима 0).
    pub const OVERCURRENT: u8 = 1 << 0;
    /// T ≥ temp_off_c → драйвер выключен (без защёлки).
    pub const OVERTEMP: u8 = 1 << 1;
    /// Сработал мягкий предел угла (момент в запрещённую сторону обнулён).
    pub const ANGLE_LIMIT: u8 = 1 << 2;
    /// Датчик угла вне 0.15..3.15 В → драйвер выключен немедленно (защёлка).
    pub const SENSOR: u8 = 1 << 3;
    /// Батарея < 3.0 В/элемент 10 мс подряд → драйвер выключен (защёлка).
    pub const BATT_LOW: u8 = 1 << 4;
    /// Батарея < 3.3 В/элемент — предупреждение.
    pub const BATT_WARN: u8 = 1 << 5;
    /// Аварийная кнопка → драйвер выключен за один тик 10 кГц (защёлка).
    pub const ESTOP: u8 = 1 << 6;
    /// Нет команд 500 мс → поддержка спадает до нуля за 200 мс.
    pub const CMD_TIMEOUT: u8 = 1 << 7;

    /// Второй байт (`fault_flags2`).
    /// Заклинивание (только режим слежения): |e| > `stall_pos_err_deg`, |I| ≥ 0.85·i_max
    /// (действующий) и |скорость| < 2 °/с дольше `stall_time_s` → ток спадает до нуля
    /// за `stall_ramp_s`, затем драйвер выключен (защёлка до команды режима 0).
    /// Флаг виден с начала спада. В режиме поддержки не срабатывает.
    pub const STALL: u8 = 1 << 0;
    /// Оценка температуры обмотки выше T_max − 25 °C (ток снижается линейно до 0 к
    /// T_max), выше T_max — драйвер выключен.
    pub const WINDING_HOT: u8 = 1 << 1;
    /// Контакт: момент сверх модели веса дольше 20 мс (режим слежения). Рука отводится на
    /// 5° и держится там до команды режима 0; драйвер не выключается, чтобы груз не упал.
    /// Если за 1 с рука не отошла — спад тока, как при заклинивании.
    pub const CONTACT: u8 = 1 << 2;

    /// Названия установленных флагов обоих байтов: `flags = fault_flags | fault_flags2 << 8`.
    pub fn names(flags: u16) -> Vec<&'static str> {
        const N: [&str; 11] = [
            "перегрузка по току",
            "перегрев",
            "предел угла",
            "датчик угла",
            "батарея разряжена",
            "батарея низкая",
            "аварийная кнопка",
            "нет связи",
            "заклинивание",
            "обмотка горячая",
            "контакт с препятствием",
        ];
        (0..11)
            .filter(|k| flags >> k & 1 != 0)
            .map(|k| N[k])
            .collect()
    }
}

/// Биты байта `state` телеметрии.
pub mod state {
    /// Биты 0..=1 — режим (0 выкл, 1 поддержка, 2 слежение).
    pub const MODE_MASK: u8 = 0x03;
    /// Драйвер включён.
    pub const DRIVER_ON: u8 = 1 << 2;
    /// Идёт снижение тока по температуре.
    pub const DERATING: u8 = 1 << 3;
    /// Последний запуск — после сброса сторожем.
    pub const AFTER_WATCHDOG: u8 = 1 << 4;
    /// Блок конфигурации не принят — работают значения по умолчанию.
    pub const DEFAULT_CONFIG: u8 = 1 << 5;
    /// Команды приходят (последняя — не старше 500 мс).
    pub const LINK_OK: u8 = 1 << 6;
}

pub const CONFIG_MAGIC: u32 = u32::from_le_bytes(*b"EXOC");
pub const CONFIG_VERSION: u32 = 4;
/// Размер блока конфигурации, байт.
pub const CONFIG_SIZE: usize = 0x98;

/// Параметры контроллера, которые хост пишет в блок конфигурации до сброса.
/// Раскладка (little-endian): 0x00 магия «EXOC», 0x04 версия = 4, 0x08 размер =
/// 0x98, 0x0C флаги (0), 0x10..=0x5C — 20 полей `f32` в порядке объявления,
/// 0x60 `wdt_timeout_ms` u32, 0x64 `test_hang_ms` u32, 0x68 резерв (0),
/// 0x6C..=0x80 — 6 полей `f32` версии 2 (тепло обмотки, заклинивание),
/// 0x84, 0x88 — 2 поля `f32` версии 3 (`stall_pos_err_deg`, `stall_ramp_s`),
/// 0x8C, 0x90 — 2 поля `f32` версии 4 (`speed_limit_dps`, `contact_torque_nm`),
/// 0x94 CRC-32 байтов 0x00..0x94. Блоки младших версий прошивка отвергает (значения по умолчанию).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ExoConfig {
    /// Постоянная момента мотора, Н·м/А.
    pub kt_nm_per_a: f32,
    /// Передаточное число редуктора.
    pub gear_ratio: f32,
    /// КПД редуктора (0..1].
    pub gear_efficiency: f32,
    /// Предел тока мотора, А.
    pub i_max_a: f32,
    /// Порог защиты по току, А.
    pub i_trip_a: f32,
    /// Число последовательных элементов батареи.
    pub cells_series: f32,
    /// Масса предплечья с кистью, кг.
    pub forearm_mass_kg: f32,
    /// Расстояние от локтя до центра масс предплечья, м.
    pub forearm_com_m: f32,
    /// Расстояние от локтя до груза в кисти, м.
    pub hand_dist_m: f32,
    pub angle_min_deg: f32,
    pub angle_max_deg: f32,
    /// Начало снижения тока по температуре, °C.
    pub temp_derate_c: f32,
    /// Выключение по температуре, °C.
    pub temp_off_c: f32,
    /// ПИД по положению: А/°.
    pub pos_kp: f32,
    /// А/(°·с).
    pub pos_ki: f32,
    /// А·с/°.
    pub pos_kd: f32,
    /// ПИ по току: В/А.
    pub cur_kp: f32,
    /// В/(А·с).
    pub cur_ki: f32,
    /// Коэффициент обратного расчёта антинасыщения, 1/с.
    pub cur_kaw: f32,
    /// Номинальное напряжение шины, В.
    pub v_bus_nominal: f32,
    /// Тайм-аут сторожа, мс.
    pub wdt_timeout_ms: u32,
    /// Тест: «зависнуть» основным циклом через столько мс после включения (0 — нет).
    pub test_hang_ms: u32,
    /// Сопротивление обмотки (межфазное), Ом, при `motor_r_ref_c`.
    pub motor_r_ohm: f32,
    /// Температура, при которой задано `motor_r_ohm`, °C.
    pub motor_r_ref_c: f32,
    /// Тепловое сопротивление «обмотка → корпус», К/Вт.
    pub rth_wh_k_per_w: f32,
    /// Тепловая постоянная времени обмотки, с (ёмкость C_w = τ_w/R_wh — трактовка maxon).
    pub tau_w_s: f32,
    /// Допустимая температура обмотки, °C.
    pub t_winding_max_c: f32,
    /// Время у предела тока без движения до срабатывания защиты от заклинивания, с.
    pub stall_time_s: f32,
    /// Заклинивание считается, только если ошибка позиции (режим 2) больше, °.
    pub stall_pos_err_deg: f32,
    /// Время спада тока до нуля после срабатывания защиты от заклинивания, с.
    pub stall_ramp_s: f32,
    /// Предел скорости траектории в режиме слежения, °/с (0 — без предела).
    pub speed_limit_dps: f32,
    /// Лишний момент сверх модели веса, по которому распознаётся контакт, Н·м (0 — выключено).
    pub contact_torque_nm: f32,
}

impl Default for ExoConfig {
    /// Значения по умолчанию — те же, что зашиты в прошивку («оценка» для
    /// экзоскелета локтя: мотор 0.05 Н·м/А, редуктор 100:1 с КПД 0.8, батарея 6S,
    /// предплечье с кистью 1.65 кг по таблицам Winter, 2009).
    fn default() -> Self {
        ExoConfig {
            kt_nm_per_a: 0.05,
            gear_ratio: 100.0,
            gear_efficiency: 0.8,
            i_max_a: 8.0,
            i_trip_a: 12.0,
            cells_series: 6.0,
            forearm_mass_kg: 1.65,
            forearm_com_m: 0.16,
            hand_dist_m: 0.35,
            angle_min_deg: 5.0,
            angle_max_deg: 140.0,
            temp_derate_c: 70.0,
            temp_off_c: 90.0,
            pos_kp: 0.15,
            pos_ki: 0.5,
            pos_kd: 0.012,
            cur_kp: 1.8,
            cur_ki: 2600.0,
            cur_kaw: 2000.0,
            v_bus_nominal: 22.2,
            wdt_timeout_ms: 50,
            test_hang_ms: 0,
            motor_r_ohm: 0.844,
            motor_r_ref_c: 25.0,
            rth_wh_k_per_w: 1.82,
            tau_w_s: 54.3,
            t_winding_max_c: 125.0,
            stall_time_s: 1.5,
            stall_pos_err_deg: 3.0,
            stall_ramp_s: 0.2,
            speed_limit_dps: 40.0,
            contact_torque_nm: 14.0,
        }
    }
}

/// CRC-32 IEEE (как `vmcu_hal::crc32` в прошивке).
pub fn crc32(data: &[u8]) -> u32 {
    let mut c = 0xffff_ffffu32;
    for &b in data {
        c ^= b as u32;
        for _ in 0..8 {
            c = if c & 1 != 0 {
                (c >> 1) ^ 0xedb8_8320
            } else {
                c >> 1
            };
        }
    }
    !c
}

impl ExoConfig {
    /// Согласованный набор для привода maxon EC 90 flat 260 Вт 48 В (500267) +
    /// Harmonic Drive CSG-25-50 (как `sim_em::motor::MotorParams::maxon_ec90flat_260w_hd_csg25_50`):
    /// Kt = 0.231 Н·м/А, N = 50, η = 0.85 («оценка»), R = 0.844 Ом при 25 °C,
    /// L = 1.07 мГн, тепло — данные maxon. Батарея стенда 6S (24 В; делитель /11
    /// измеряет до 36 В). Токовый ПИ на полосу ≈ 700 Гц: Kp = L·ω, Ki = R·ω.
    /// Позиционные усиления — под приведённую инерцию ≈ 1.4 кг·м² (ω ≈ 7.7 рад/с,
    /// ζ ≈ 0.7; «оценка»).
    pub fn maxon_ec90flat_hd50() -> ExoConfig {
        let w = 2.0 * std::f32::consts::PI * 700.0;
        ExoConfig {
            kt_nm_per_a: 0.231,
            gear_ratio: 50.0,
            gear_efficiency: 0.85,
            i_max_a: 8.1,
            i_trip_a: 12.0,
            cells_series: 6.0,
            cur_kp: 1.07e-3 * w,
            cur_ki: 0.844 * w,
            pos_kp: 0.15,
            pos_ki: 0.3,
            pos_kd: 0.027,
            v_bus_nominal: 22.2,
            ..ExoConfig::default()
        }
    }

    fn floats(&self) -> [f32; 20] {
        [
            self.kt_nm_per_a,
            self.gear_ratio,
            self.gear_efficiency,
            self.i_max_a,
            self.i_trip_a,
            self.cells_series,
            self.forearm_mass_kg,
            self.forearm_com_m,
            self.hand_dist_m,
            self.angle_min_deg,
            self.angle_max_deg,
            self.temp_derate_c,
            self.temp_off_c,
            self.pos_kp,
            self.pos_ki,
            self.pos_kd,
            self.cur_kp,
            self.cur_ki,
            self.cur_kaw,
            self.v_bus_nominal,
        ]
    }

    /// Байты блока конфигурации (0x90 байт) для [`Mcu::write_config`].
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut b = Vec::with_capacity(CONFIG_SIZE);
        b.extend_from_slice(&CONFIG_MAGIC.to_le_bytes());
        b.extend_from_slice(&CONFIG_VERSION.to_le_bytes());
        b.extend_from_slice(&(CONFIG_SIZE as u32).to_le_bytes());
        b.extend_from_slice(&0u32.to_le_bytes());
        for f in self.floats() {
            b.extend_from_slice(&f.to_le_bytes());
        }
        b.extend_from_slice(&self.wdt_timeout_ms.to_le_bytes());
        b.extend_from_slice(&self.test_hang_ms.to_le_bytes());
        b.extend_from_slice(&0u32.to_le_bytes());
        for f in [
            self.motor_r_ohm,
            self.motor_r_ref_c,
            self.rth_wh_k_per_w,
            self.tau_w_s,
            self.t_winding_max_c,
            self.stall_time_s,
            self.stall_pos_err_deg,
            self.stall_ramp_s,
            self.speed_limit_dps,
            self.contact_torque_nm,
        ] {
            b.extend_from_slice(&f.to_le_bytes());
        }
        let crc = crc32(&b);
        b.extend_from_slice(&crc.to_le_bytes());
        debug_assert_eq!(b.len(), CONFIG_SIZE);
        b
    }

    /// Разбор блока (проверка магии, версии, размера и CRC).
    pub fn from_bytes(b: &[u8]) -> Option<ExoConfig> {
        if b.len() < CONFIG_SIZE {
            return None;
        }
        let u = |o: usize| u32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]]);
        let f = |o: usize| f32::from_bits(u(o));
        if u(0) != CONFIG_MAGIC
            || u(4) != CONFIG_VERSION
            || u(8) as usize != CONFIG_SIZE
            || crc32(&b[..CONFIG_SIZE - 4]) != u(CONFIG_SIZE - 4)
        {
            return None;
        }
        Some(ExoConfig {
            kt_nm_per_a: f(0x10),
            gear_ratio: f(0x14),
            gear_efficiency: f(0x18),
            i_max_a: f(0x1c),
            i_trip_a: f(0x20),
            cells_series: f(0x24),
            forearm_mass_kg: f(0x28),
            forearm_com_m: f(0x2c),
            hand_dist_m: f(0x30),
            angle_min_deg: f(0x34),
            angle_max_deg: f(0x38),
            temp_derate_c: f(0x3c),
            temp_off_c: f(0x40),
            pos_kp: f(0x44),
            pos_ki: f(0x48),
            pos_kd: f(0x4c),
            cur_kp: f(0x50),
            cur_ki: f(0x54),
            cur_kaw: f(0x58),
            v_bus_nominal: f(0x5c),
            wdt_timeout_ms: u(0x60),
            test_hang_ms: u(0x64),
            motor_r_ohm: f(0x6c),
            motor_r_ref_c: f(0x70),
            rth_wh_k_per_w: f(0x74),
            tau_w_s: f(0x78),
            t_winding_max_c: f(0x7c),
            stall_time_s: f(0x80),
            stall_pos_err_deg: f(0x84),
            stall_ramp_s: f(0x88),
            speed_limit_dps: f(0x8c),
            contact_torque_nm: f(0x90),
        })
    }

    /// Предел тока по оценке температуры обмотки «вручную»: i_max·(T_max − T)/25
    /// в полосе T_max − 25..T_max, выше — 0.
    pub fn winding_current_limit(&self, t_winding_c: f64) -> f64 {
        let t_max = self.t_winding_max_c as f64;
        let k = ((t_max - t_winding_c) / 25.0).clamp(0.0, 1.0);
        self.i_max_a as f64 * k
    }

    /// Момент на выходе редуктора на 1 А тока мотора, Н·м/А: Kt·N·η.
    pub fn torque_per_amp(&self) -> f64 {
        self.kt_nm_per_a as f64 * self.gear_ratio as f64 * self.gear_efficiency as f64
    }

    /// Ток гравитационной поддержки «вручную» (второй способ для проверки прошивки):
    /// τ = assist·(m_f·g·r_c + m_груз·g·L)·sin θ, i = τ/(Kt·N·η), ограничение ±i_max.
    pub fn gravity_assist_current(&self, assist_pct: f64, payload_kg: f64, theta_deg: f64) -> f64 {
        let g = 9.81;
        let tau = assist_pct / 100.0
            * (self.forearm_mass_kg as f64 * g * self.forearm_com_m as f64
                + payload_kg * g * self.hand_dist_m as f64)
            * theta_deg.to_radians().sin();
        let i_max = self.i_max_a as f64;
        (tau / self.torque_per_amp()).clamp(-i_max, i_max)
    }
}

/// Режим работы.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum ExoMode {
    Off = 0,
    GravityAssist = 1,
    PositionTrack = 2,
}

/// Команда телефон → экзоскелет (8 байт): `0xC1, режим u8, поддержка % u8,
/// цель°×10 u16 LE, груз кг×10 u16 LE, seq u8`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ExoCommand {
    pub mode: ExoMode,
    pub assist_pct: u8,
    pub target_deg: f64,
    pub payload_kg: f64,
    pub seq: u8,
}

pub const CMD_TAG: u8 = 0xc1;
pub const CMD_LEN: usize = 8;
pub const TELEMETRY_TAG: u8 = 0x7e;
/// 14 байт: к формату версии 1 (13 байт) в конец добавлен `fault_flags2`.
pub const TELEMETRY_LEN: usize = 14;
/// Период телеметрии, с.
pub const TELEMETRY_PERIOD_S: f64 = 0.02;

fn scale_u16(x: f64, k: f64) -> u16 {
    (x * k).round().clamp(0.0, 65535.0) as u16
}

impl ExoCommand {
    pub fn encode(&self) -> [u8; CMD_LEN] {
        let t = scale_u16(self.target_deg, 10.0).to_le_bytes();
        let p = scale_u16(self.payload_kg, 10.0).to_le_bytes();
        [
            CMD_TAG,
            self.mode as u8,
            self.assist_pct,
            t[0],
            t[1],
            p[0],
            p[1],
            self.seq,
        ]
    }

    pub fn decode(b: &[u8]) -> Option<ExoCommand> {
        if b.len() != CMD_LEN || b[0] != CMD_TAG {
            return None;
        }
        let mode = match b[1] {
            0 => ExoMode::Off,
            1 => ExoMode::GravityAssist,
            2 => ExoMode::PositionTrack,
            _ => return None,
        };
        Some(ExoCommand {
            mode,
            assist_pct: b[2],
            target_deg: u16::from_le_bytes([b[3], b[4]]) as f64 / 10.0,
            payload_kg: u16::from_le_bytes([b[5], b[6]]) as f64 / 10.0,
            seq: b[7],
        })
    }
}

/// Телеметрия экзоскелет → телефон, каждые 20 мс (14 байт): `0x7E, state u8,
/// угол°×10 u16, ток А×100 i16, температура корпуса °C×10 i16, Vbat×100 u16,
/// fault_flags u8, seq u8, last_cmd_seq u8, fault_flags2 u8` (всё LE).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ExoTelemetry {
    pub state: u8,
    pub angle_deg: f64,
    pub current_a: f64,
    pub temp_c: f64,
    pub vbat_v: f64,
    pub fault_flags: u8,
    pub seq: u8,
    pub last_cmd_seq: u8,
    /// Второй байт флагов ([`fault::STALL`], [`fault::WINDING_HOT`]).
    pub fault_flags2: u8,
}

impl ExoTelemetry {
    pub fn decode(b: &[u8]) -> Option<ExoTelemetry> {
        if b.len() != TELEMETRY_LEN || b[0] != TELEMETRY_TAG {
            return None;
        }
        Some(ExoTelemetry {
            state: b[1],
            angle_deg: u16::from_le_bytes([b[2], b[3]]) as f64 / 10.0,
            current_a: i16::from_le_bytes([b[4], b[5]]) as f64 / 100.0,
            temp_c: i16::from_le_bytes([b[6], b[7]]) as f64 / 10.0,
            vbat_v: u16::from_le_bytes([b[8], b[9]]) as f64 / 100.0,
            fault_flags: b[10],
            seq: b[11],
            last_cmd_seq: b[12],
            fault_flags2: b[13],
        })
    }

    pub fn encode(&self) -> [u8; TELEMETRY_LEN] {
        let a = scale_u16(self.angle_deg, 10.0).to_le_bytes();
        let i = ((self.current_a * 100.0).round().clamp(-32768.0, 32767.0) as i16).to_le_bytes();
        let t = ((self.temp_c * 10.0).round().clamp(-32768.0, 32767.0) as i16).to_le_bytes();
        let v = scale_u16(self.vbat_v, 100.0).to_le_bytes();
        [
            TELEMETRY_TAG,
            self.state,
            a[0],
            a[1],
            i[0],
            i[1],
            t[0],
            t[1],
            v[0],
            v[1],
            self.fault_flags,
            self.seq,
            self.last_cmd_seq,
            self.fault_flags2,
        ]
    }

    pub fn mode(&self) -> u8 {
        self.state & state::MODE_MASK
    }
    pub fn driver_on(&self) -> bool {
        self.state & state::DRIVER_ON != 0
    }
    /// Флаг первого байта.
    pub fn has(&self, flag: u8) -> bool {
        self.fault_flags & flag != 0
    }
    /// Флаг второго байта.
    pub fn has2(&self, flag: u8) -> bool {
        self.fault_flags2 & flag != 0
    }
    /// Оба байта флагов: `fault_flags | fault_flags2 << 8` (для [`fault::names`]).
    pub fn all_faults(&self) -> u16 {
        self.fault_flags as u16 | (self.fault_flags2 as u16) << 8
    }
}

/// Диагностический блок прошивки (символ `EXO_DIAG`, обновляется каждую 1 мс) —
/// то, что инженер читает отладчиком: оценка обмотки, предел тока, счётчик
/// заклинивания. Раскладка `#[repr(C)]` — в `MEMORY_MAP.md`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ExoDiag {
    pub winding_est_c: f64,
    pub housing_c: f64,
    pub i2_mean_a2: f64,
    pub i_limit_a: f64,
    pub stall_ms: u32,
    pub stall_vel_dps: f64,
    /// Оценка обмотки восстановлена после сброса сторожем.
    pub winding_retained: bool,
    /// Заклинивание: 0 — нет, 1 — идёт спад тока, 2 — защёлка.
    pub stall_state: u32,
}

pub const DIAG_MAGIC: u32 = 0x4741_4944;

impl ExoDiag {
    /// Прочитать блок из ОЗУ МК (`None` — нет символа или блок ещё не записан).
    pub fn read(mcu: &Mcu) -> Option<ExoDiag> {
        let a = mcu.symbol("EXO_DIAG")?;
        if mcu.read_u32(a)? != DIAG_MAGIC {
            return None;
        }
        let f = |o: u32| mcu.read_f32(a + o).map(|v| v as f64);
        Some(ExoDiag {
            winding_est_c: f(4)?,
            housing_c: f(8)?,
            i2_mean_a2: f(12)?,
            i_limit_a: f(16)?,
            stall_ms: mcu.read_u32(a + 20)?,
            stall_vel_dps: f(24)?,
            winding_retained: mcu.read_u32(a + 28)? != 0,
            stall_state: mcu.read_u32(a + 32)?,
        })
    }
}

/// Выходы МК, нужные силовой части: (заполнение 0..1, направление «сгибание», драйвер включён).
pub fn read_drive(mcu: &Mcu) -> (f64, bool, bool) {
    (
        mcu.pwm_duty(pins::PWM_MOTOR),
        mcu.gpio_output(pins::GPIO_DIR),
        mcu.gpio_output(pins::GPIO_ENABLE),
    )
}

/// Подать на АЦП показания датчиков по шкалам контракта.
pub fn write_sensors(mcu: &mut Mcu, theta_deg: f64, current_a: f64, temp_c: f64, vbat_v: f64) {
    mcu.set_adc_volts(pins::ADC_ANGLE, sensors::angle_volts(theta_deg));
    mcu.set_adc_volts(pins::ADC_CURRENT, sensors::current_volts(current_a));
    mcu.set_adc_volts(pins::ADC_TEMP, sensors::temp_volts(temp_c));
    mcu.set_adc_volts(pins::ADC_VBAT, sensors::battery_volts(vbat_v));
}
