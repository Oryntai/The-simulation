//! Блок конфигурации (хост пишет по 0x5000_0000 до сброса). Раскладка — в
//! `crates/sim-mcu/MEMORY_MAP.md`, хостовая сторона — `sim_mcu::exo::ExoConfig`.
//! Блок принимается, только если совпали магия, версия, размер и CRC-32 и все
//! величины в разумных пределах; иначе — значения по умолчанию ([`Config::DEFAULT`]).

use vmcu_hal::config;

pub const MAGIC: u32 = u32::from_le_bytes(*b"EXOC");
pub const VERSION: u32 = 4;
pub const SIZE: usize = 0x98;

/// Параметры контроллера. Единицы — в именах полей.
#[derive(Clone, Copy)]
pub struct Config {
    pub kt_nm_per_a: f32,
    pub gear_ratio: f32,
    pub gear_efficiency: f32,
    pub i_max_a: f32,
    pub i_trip_a: f32,
    pub cells_series: f32,
    pub forearm_mass_kg: f32,
    pub forearm_com_m: f32,
    pub hand_dist_m: f32,
    pub angle_min_deg: f32,
    pub angle_max_deg: f32,
    pub temp_derate_c: f32,
    pub temp_off_c: f32,
    /// Позиционный ПИД: А/°, А/(°·с), А·с/°.
    pub pos_kp: f32,
    pub pos_ki: f32,
    pub pos_kd: f32,
    /// Токовый ПИ: В/А, В/(А·с); обратный расчёт антинасыщения, 1/с.
    pub cur_kp: f32,
    pub cur_ki: f32,
    pub cur_kaw: f32,
    pub v_bus_nominal: f32,
    pub wdt_timeout_ms: u32,
    /// Тест: через столько мс после включения питания основной цикл «зависает»
    /// (0 — выключено). После сброса сторожем не повторяется.
    pub test_hang_ms: u32,
    /// Тепловая модель обмотки (версия 2): сопротивление обмотки, Ом, при
    /// температуре `motor_r_ref_c`, °C.
    pub motor_r_ohm: f32,
    pub motor_r_ref_c: f32,
    /// Тепловое сопротивление «обмотка → корпус», К/Вт.
    pub rth_wh_k_per_w: f32,
    /// Тепловая постоянная времени обмотки, с (C_w = τ_w / R_wh).
    pub tau_w_s: f32,
    /// Допустимая температура обмотки, °C.
    pub t_winding_max_c: f32,
    /// Защита от заклинивания: время у предела тока без движения, с.
    pub stall_time_s: f32,
    /// Версия 3: заклинивание считается только при ошибке позиции больше, °.
    pub stall_pos_err_deg: f32,
    /// Версия 3: время спада тока до нуля после срабатывания, с.
    pub stall_ramp_s: f32,
    /// Версия 4: предел скорости траектории в режиме слежения, °/с (0 — без предела).
    pub speed_limit_dps: f32,
    /// Версия 4: лишний момент, по которому распознаётся контакт, Н·м (0 — выключено).
    pub contact_torque_nm: f32,
}

impl Config {
    /// Значения по умолчанию (совпадают с `ExoConfig::default()` на хосте).
    pub const DEFAULT: Config = Config {
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
        // maxon EC 90 flat 260 Вт (500267): данные maxon.
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
    };

    fn valid(&self) -> bool {
        let pos = [
            self.kt_nm_per_a,
            self.gear_ratio,
            self.gear_efficiency,
            self.i_max_a,
            self.i_trip_a,
            self.cells_series,
            self.forearm_mass_kg,
            self.forearm_com_m,
            self.hand_dist_m,
            self.v_bus_nominal,
            self.motor_r_ohm,
            self.rth_wh_k_per_w,
            self.tau_w_s,
            self.t_winding_max_c,
            self.stall_time_s,
            self.stall_pos_err_deg,
            self.stall_ramp_s,
        ];
        let all = [
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
            self.motor_r_ref_c + 273.15,
            self.speed_limit_dps,
            self.contact_torque_nm,
        ];
        pos.iter().all(|v| v.is_finite() && *v > 0.0)
            && all.iter().all(|v| v.is_finite() && *v >= 0.0)
            && self.gear_efficiency <= 1.0
            && self.angle_min_deg < self.angle_max_deg
            && self.temp_derate_c < self.temp_off_c
            && self.wdt_timeout_ms >= 1
            && self.t_winding_max_c > 25.0
    }
}

/// Прочитать блок. Возвращает (параметры, взяты_из_блока).
pub fn load() -> (Config, bool) {
    let b = config::bytes();
    if config::u32_at(0) != MAGIC
        || config::u32_at(4) != VERSION
        || config::u32_at(8) as usize != SIZE
        || vmcu_hal::crc32(&b[..SIZE - 4]) != config::u32_at(SIZE - 4)
    {
        return (Config::DEFAULT, false);
    }
    let f = config::f32_at;
    let c = Config {
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
        wdt_timeout_ms: config::u32_at(0x60),
        test_hang_ms: config::u32_at(0x64),
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
    };
    if c.valid() {
        (c, true)
    } else {
        (Config::DEFAULT, false)
    }
}
