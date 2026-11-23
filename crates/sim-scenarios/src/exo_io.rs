//! «Провода» между физикой и прошивкой экзоскелета локтя: что видят входы МК (АЦП, кнопка)
//! и как телефон говорит с экзо по радио. Это контракт ввода-вывода прошивки `exo-ctrl`
//! (sim/firmware/exo-ctrl, описан в crates/sim-mcu/MEMORY_MAP.md); здесь — его независимая
//! запись со стороны стенда, сверяется тестом с кодировщиками sim-mcu.

/// Напряжение датчика угла: V = 0.3 + 2.7·θ/150 (θ — сгибание, °).
pub fn angle_sensor_volts(theta_deg: f64) -> f64 {
    0.3 + 2.7 * theta_deg / 150.0
}

/// Датчик тока: V = 1.65 + 0.1·I (А, + — момент сгибания).
pub fn current_sensor_volts(amps: f64) -> f64 {
    (1.65 + 0.1 * amps).clamp(0.0, 3.3)
}

/// Датчик температуры (как TMP36): V = 0.5 + 0.01·T (°C).
pub fn temp_sensor_volts(t_c: f64) -> f64 {
    (0.5 + 0.01 * t_c).clamp(0.0, 3.3)
}

/// Делитель батареи 1:11.
pub fn battery_sense_volts(v_bat: f64) -> f64 {
    (v_bat / 11.0).clamp(0.0, 3.3)
}

/// Каналы АЦП.
pub const ADC_ANGLE: usize = 0;
pub const ADC_CURRENT: usize = 1;
pub const ADC_TEMP: usize = 2;
pub const ADC_VBAT: usize = 3;
/// Вход аварийной кнопки (активный низкий).
pub const GPIO_ESTOP: usize = 0;
/// Выходы: направление, разрешение драйвера, светодиод.
pub const GPIO_DIR: usize = 1;
pub const GPIO_ENABLE: usize = 2;
pub const GPIO_LED: usize = 3;
/// Канал ШИМ моста.
pub const PWM_BRIDGE: usize = 0;

/// Режим экзо по команде телефона.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    Off = 0,
    GravityAssist = 1,
    Position = 2,
}

/// Команда телефона → экзо: `0xC1, mode, assist %, target·10 (u16 LE), payload·10 (u16 LE), seq`.
pub fn encode_command(
    mode: Mode,
    assist_pct: u8,
    target_deg: f64,
    payload_kg: f64,
    seq: u8,
) -> Vec<u8> {
    let t = (target_deg * 10.0).round().clamp(0.0, 65535.0) as u16;
    let p = (payload_kg * 10.0).round().clamp(0.0, 65535.0) as u16;
    let mut v = vec![0xC1, mode as u8, assist_pct.min(100)];
    v.extend_from_slice(&t.to_le_bytes());
    v.extend_from_slice(&p.to_le_bytes());
    v.push(seq);
    v
}

/// Телеметрия экзо → телефон.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Telemetry {
    pub state: u8,
    pub angle_deg: f64,
    pub current_a: f64,
    pub temp_c: f64,
    pub vbat: f64,
    pub faults: u8,
    pub seq: u8,
    pub last_cmd_seq: u8,
    /// Второй байт флагов (бит 0 — заклинивание, бит 1 — перегрев обмотки по оценке).
    pub faults2: u8,
}

/// Разбор телеметрии (14 байт): `0x7E, state, angle·10 u16, current·100 i16, temp·10 i16,
/// vbat·100 u16, faults u8, seq u8, last_cmd_seq u8, faults2 u8` (все многобайтные — LE).
pub fn decode_telemetry(b: &[u8]) -> Option<Telemetry> {
    if b.len() != 14 || b[0] != 0x7E {
        return None;
    }
    let u16_at = |i: usize| u16::from_le_bytes([b[i], b[i + 1]]);
    let i16_at = |i: usize| i16::from_le_bytes([b[i], b[i + 1]]);
    Some(Telemetry {
        state: b[1],
        angle_deg: u16_at(2) as f64 / 10.0,
        current_a: i16_at(4) as f64 / 100.0,
        temp_c: i16_at(6) as f64 / 10.0,
        vbat: u16_at(8) as f64 / 100.0,
        faults: b[10],
        seq: b[11],
        last_cmd_seq: b[12],
        faults2: b[13],
    })
}
