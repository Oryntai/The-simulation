//! Регулятор экзоскелета.
//!
//! * Быстрый контур — прерывание таймера 10 кГц ([`Ctl::fast_tick`]): опрос
//!   аварийной кнопки, датчика угла и тока, мгновенные защиты, ПИ-регулятор тока
//!   в фиксированной точке Q16 (у RV32IMC нет FPU — так делают и на настоящих МК),
//!   выход ШИМ + направление + разрешение драйвера.
//! * Внешний контур — каждый 10-й тик, 1 кГц ([`Ctl::outer`]), в `f32` (программная
//!   плавающая точка): угол, скорость, температура, батарея, связь, режим
//!   (гравитационная поддержка или ПИД по положению) → задание тока.
//!
//! * Тепловая модель обмотки (1 кГц, [`Ctl::winding_step`]): узел «обмотка»
//!   нагревается потерями I²·R(T) и отдаёт тепло корпусу через R_wh; температура
//!   корпуса — ИЗМЕРЕННАЯ (ADC2). Это стандартная I²t-защита двигателя: датчик на
//!   корпусе отстаёт от обмотки на десятки кельвинов и минуты.
//!
//! Защиты и их флаги — см. `F_*`, `F2_*`; время реакции — в `MEMORY_MAP.md`.

use crate::config::Config;
use vmcu_hal::{adc, gpio, pwm, timer};

// Каналы и выводы (контракт с хостом — `sim_mcu::exo`).
pub const ADC_ANGLE: u32 = 0;
pub const ADC_CURRENT: u32 = 1;
pub const ADC_TEMP: u32 = 2;
pub const ADC_VBAT: u32 = 3;
pub const PIN_ESTOP: u32 = 0;
pub const PIN_DIR: u32 = 1;
pub const PIN_EN: u32 = 2;
pub const PIN_LED: u32 = 3;
pub const PWM_MOTOR: u32 = 0;

/// Частота быстрого контура и период в тиках `mtime` (16 МГц).
pub const TICK_HZ: u32 = 10_000;
pub const MTIME_PER_TICK: u64 = 1600;
const DT_TICK: f32 = 1.0 / TICK_HZ as f32;
const DT_OUTER: f32 = 1e-3;

/// Флаги неисправностей (байт `fault_flags` телеметрии).
pub const F_OVERCURRENT: u8 = 1 << 0;
pub const F_OVERTEMP: u8 = 1 << 1;
pub const F_ANGLE_LIMIT: u8 = 1 << 2;
pub const F_SENSOR: u8 = 1 << 3;
pub const F_BATT_LOW: u8 = 1 << 4;
pub const F_BATT_WARN: u8 = 1 << 5;
pub const F_ESTOP: u8 = 1 << 6;
pub const F_CMD_TIMEOUT: u8 = 1 << 7;
/// Второй байт флагов (`fault_flags2`, байт 13 телеметрии).
/// Заклинивание: в режиме слежения цель далеко (|e| > stall_pos_err_deg), ток у
/// предела, сустав не движется дольше stall_time_s → спад тока за stall_ramp_s,
/// затем выключение и защёлка. Флаг виден с начала спада.
pub const F2_STALL: u8 = 1 << 0;
/// Оценка температуры обмотки выше T_max − 25 °C (снижение тока), выше T_max — выкл.
pub const F2_WINDING_HOT: u8 = 1 << 1;
/// Контакт: привод упёрся (момент сверх модели веса). Рука отводится назад и держится
/// там до команды режима 0. Драйвер не выключается, чтобы груз не упал.
pub const F2_CONTACT: u8 = 1 << 2;

/// Траектория режима слежения: предел ускорения, °/с², и наклон торможения у цели, 1/с.
const REF_ACCEL_DPS2: f32 = 100.0;
const REF_GAIN_PER_S: f32 = 5.0;
/// Контакт подтверждается за столько мс подряд (короткий бросок тока при трогании — не контакт).
const CONTACT_CONFIRM_MS: u32 = 20;
/// На сколько отвести руку от препятствия, °.
const CONTACT_BACKOFF_DEG: f32 = 5.0;
/// Если за это время рука не отошла, сустав заклинило: включается спад тока.
const CONTACT_RETREAT_MS: u32 = 1000;
const CONTACT_RETREAT_TOL_DEG: f32 = 2.0;
/// Сустав считается стоящим при скорости ниже этой, °/с.
const CONTACT_STILL_DPS: f32 = 2.0;

/// Предел заполнения ШИМ 0.95 в Q16.
const DUTY_MAX_Q16: i32 = 62_259;
/// Ток в Q16 (А·65536) на код АЦП: (3.3/4096 В)/(0.1 В/А)·65536 = 528 ровно.
const I_Q16_PER_CODE: i32 = 528;
/// Середина шкалы тока (1.65 В) — код 2048.
const I_ZERO_CODE: i32 = 2048;
/// Исправный датчик угла: 0.15 В ≤ V ≤ 3.15 В → коды 186..=3909.
const SENSOR_LO_CODE: u32 = 186;
const SENSOR_HI_CODE: u32 = 3909;
/// Перегрузка по току: дольше 2 мс = больше 20 отсчётов подряд.
const OC_TICKS: u32 = 20;
/// Низкая батарея: 10 мс подряд ниже 3.0 В/элемент.
const BATT_LOW_MS: u32 = 10;
const BATT_LOW_V_CELL: f32 = 3.0;
const BATT_WARN_V_CELL: f32 = 3.3;
/// Потеря связи: 500 мс без команды → спад поддержки до нуля за 200 мс.
const CMD_TIMEOUT_MS: u32 = 500;
const RAMP_MS: f32 = 200.0;
/// Фильтр скорости: постоянная 10 мс → α = Ts/(Ts+Tf).
const VEL_ALPHA: f32 = 1.0 / 11.0;
const G: f32 = 9.81;
/// Интеграл позиционного ПИД копится только вблизи цели (|e| < 5°): так большой
/// скачок задания не «накачивает» интегратор (условное интегрирование).
const POS_INTEG_BAND_DEG: f32 = 5.0;
/// Угол на единицу суммы 10 кодов: (3.3/4096)·(150/2.7)/10.
const DEG_PER_CODE_SUM10: f32 = 3.3 / 4096.0 * (150.0 / 2.7) * 0.1;
const DEG_OFFSET: f32 = 0.3 * (150.0 / 2.7);
/// Квадрат тока на квадрат кода АЦП: (3.3/4096/0.1)² А².
const I2_PER_CODE2: f32 = (33.0 / 4096.0) * (33.0 / 4096.0);
/// Температурный коэффициент сопротивления меди, 1/К.
const ALPHA_CU: f32 = 0.003_93;
/// Ширина полосы снижения тока по обмотке: от T_max − 25 до T_max.
const WINDING_DERATE_BAND_C: f32 = 25.0;
/// Заклинивание: |I| ≥ 0.85·i_max (действующий) и |скорость| < 2 °/с — и только
/// когда движение ТРЕБУЕТСЯ (режим 2, цель далеко). Удержание груза неподвижно у
/// предела тока в режиме поддержки — не заклинивание: там обмотку бережёт
/// тепловая модель (плавное снижение тока), а не отключение.
const STALL_CURRENT_FRAC: f32 = 0.85;
const STALL_SPEED_DPS: f32 = 2.0;
/// Скорость для обнаружения заклинивания — с фильтром 100 мс (шум датчика угла
/// в 1 младший разряд не сбрасывает счёт).
const STALL_VEL_ALPHA: f32 = 1.0 / 101.0;

/// |x| без libm.
fn fabs(x: f32) -> f32 {
    f32::from_bits(x.to_bits() & 0x7fff_ffff)
}

/// Диагностический блок для стенда и отладчика (как переменные, которые читает
/// отладчик по SWD/JTAG): хост читает его по символу `EXO_DIAG`. Обновляется
/// каждую 1 мс. Раскладка — в `MEMORY_MAP.md`.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct Diag {
    /// 0x4741_4944 («DIAG»).
    pub magic: u32,
    pub winding_est_c: f32,
    pub housing_c: f32,
    /// Средний за 1 мс квадрат тока, А².
    pub i2_mean_a2: f32,
    /// Действующий предел тока, А.
    pub i_limit_a: f32,
    /// Сколько мс подряд выполнено условие заклинивания.
    pub stall_ms: u32,
    pub stall_vel_dps: f32,
    /// 1 — оценка обмотки восстановлена после сброса сторожем.
    pub winding_retained: u32,
    /// Заклинивание: 0 — нет, 1 — идёт спад тока, 2 — защёлка (драйвер выключен).
    pub stall_state: u32,
}

#[unsafe(no_mangle)]
pub static mut EXO_DIAG: Diag = Diag {
    magic: 0,
    winding_est_c: 0.0,
    housing_c: 0.0,
    i2_mean_a2: 0.0,
    i_limit_a: 0.0,
    stall_ms: 0,
    stall_vel_dps: 0.0,
    winding_retained: 0,
    stall_state: 0,
};

/// Оценка температуры обмотки в ОЗУ, которое не обнуляется стартовым кодом
/// (секция `.noinit`): после сброса сторожем оценка продолжается, а не начинается
/// заново с температуры корпуса. [магия, биты f32, инверсия битов].
#[unsafe(link_section = ".noinit")]
static mut WINDING_KEEP: [u32; 3] = [0; 3];
const KEEP_MAGIC: u32 = 0x5457_4b50;

fn keep_store(tw: f32) {
    let p = &raw mut WINDING_KEEP as *mut u32;
    // SAFETY: статический массив из 3 слов, ядро одно.
    unsafe {
        p.write_volatile(KEEP_MAGIC);
        p.add(1).write_volatile(tw.to_bits());
        p.add(2).write_volatile(!tw.to_bits());
    }
}

/// Сохранённая оценка, если она целая.
pub fn keep_load() -> Option<f32> {
    let p = &raw const WINDING_KEEP as *const u32;
    // SAFETY: см. выше.
    let (m, b, nb) = unsafe {
        (
            p.read_volatile(),
            p.add(1).read_volatile(),
            p.add(2).read_volatile(),
        )
    };
    let v = f32::from_bits(b);
    (m == KEEP_MAGIC && nb == !b && v.is_finite()).then_some(v)
}

/// sin(угол в градусах) для углов −180..180°: приведение к [−90°, 90°] и ряд
/// Тейлора до x⁹ (погрешность < 4·10⁻⁶).
pub fn sin_deg(deg: f32) -> f32 {
    let mut x = deg.clamp(-180.0, 180.0) * (core::f32::consts::PI / 180.0);
    let half = core::f32::consts::FRAC_PI_2;
    if x > half {
        x = core::f32::consts::PI - x;
    } else if x < -half {
        x = -core::f32::consts::PI - x;
    }
    // Деление в программной плавающей точке дорого — только умножения.
    let x2 = x * x;
    let t = 1.0 - x2 * (1.0 / 72.0);
    let t = 1.0 - x2 * (1.0 / 42.0) * t;
    let t = 1.0 - x2 * (1.0 / 20.0) * t;
    x * (1.0 - x2 * (1.0 / 6.0) * t)
}

/// Округление к ближайшему (без libm).
fn round_i32(x: f32) -> i32 {
    if x >= 0.0 {
        (x + 0.5) as i32
    } else {
        (x - 0.5) as i32
    }
}

pub struct Ctl {
    pub cfg: Config,
    pub cfg_from_block: bool,
    pub after_watchdog: bool,
    /// Счётчик тиков 10 кГц с запуска.
    pub tick: u32,
    next_cmp: u64,
    // Быстрый контур (Q16).
    kp_q16: i32,
    ki_q16: i32,
    kaw_q16: i32,
    integ_q16: i32,
    i_ref_q16: i32,
    i_trip_q16: i32,
    oc_count: u32,
    angle_sum: u32,
    cur_sum: i32,
    n_sum: u32,
    driver_on: bool,
    dir_flex: bool,
    // Команда.
    mode: u8,
    assist_pct: u8,
    target_deg: f32,
    payload_kg: f32,
    pub last_cmd_seq: u8,
    ms_since_cmd: u32,
    // Внешний контур.
    angle_deg: f32,
    vel_dps: f32,
    current_a: f32,
    temp_c: f32,
    vbat_v: f32,
    pos_integ: f32,
    batt_low_count: u32,
    first_outer: bool,
    /// Защёлкнутые неисправности (сброс — командой режима 0).
    latched: u8,
    /// Текущие незащёлкиваемые флаги.
    live: u8,
    /// Все флаги, бывшие с прошлой телеметрии.
    seen: u8,
    derating: bool,
    /// Перегрев корпуса или обмотки выше предела — драйвер выключен.
    blocked: bool,
    // Второй байт флагов.
    latched2: u8,
    live2: u8,
    seen2: u8,
    // Тепловая модель обмотки.
    i2_sum: u32,
    tw: f32,
    tw_comp: f32,
    tw_ready: bool,
    tw_retained: bool,
    inv_rwh: f32,
    dt_over_cw: f32,
    // Заклинивание.
    stall_vel: f32,
    stall_ms: u32,
    stall_limit_ms: u32,
    /// Спад тока после срабатывания: идёт ли и сколько мс прошло.
    stall_ramping: bool,
    stall_ramp_ms: u32,
    stall_ramp_total_ms: u32,
    inv_stall_ramp_ms: f32,
    // Траектория режима слежения.
    ref_deg: f32,
    ref_vel: f32,
    ref_valid: bool,
    // Контакт с препятствием.
    contact_hold: bool,
    contact_target: f32,
    contact_ms: u32,
    contact_hold_ms: u32,
    // Заранее посчитанные величины (деление в программной плавающей точке дорого).
    inv_cells: f32,
    inv_torque_per_amp: f32,
    inv_temp_span: f32,
    /// m_f·g·r_c, Н·м.
    grav_forearm: f32,
    /// g·L, Н·м/кг.
    grav_per_payload: f32,
    pos_integ_lim: f32,
}

impl Ctl {
    pub const fn new() -> Ctl {
        Ctl {
            cfg: Config::DEFAULT,
            cfg_from_block: false,
            after_watchdog: false,
            tick: 0,
            next_cmp: 0,
            kp_q16: 0,
            ki_q16: 0,
            kaw_q16: 0,
            integ_q16: 0,
            i_ref_q16: 0,
            i_trip_q16: 0,
            oc_count: 0,
            angle_sum: 0,
            cur_sum: 0,
            n_sum: 0,
            driver_on: false,
            dir_flex: false,
            mode: 0,
            assist_pct: 0,
            target_deg: 0.0,
            payload_kg: 0.0,
            last_cmd_seq: 0,
            ms_since_cmd: 0,
            angle_deg: 0.0,
            vel_dps: 0.0,
            current_a: 0.0,
            temp_c: 0.0,
            vbat_v: 0.0,
            pos_integ: 0.0,
            batt_low_count: 0,
            first_outer: true,
            latched: 0,
            live: 0,
            seen: 0,
            derating: false,
            blocked: false,
            latched2: 0,
            live2: 0,
            seen2: 0,
            i2_sum: 0,
            tw: 0.0,
            tw_comp: 0.0,
            tw_ready: false,
            tw_retained: false,
            inv_rwh: 0.0,
            dt_over_cw: 0.0,
            stall_vel: 0.0,
            stall_ms: 0,
            stall_limit_ms: 0,
            stall_ramping: false,
            stall_ramp_ms: 0,
            stall_ramp_total_ms: 0,
            inv_stall_ramp_ms: 0.0,
            ref_deg: 0.0,
            ref_vel: 0.0,
            ref_valid: false,
            contact_hold: false,
            contact_target: 0.0,
            contact_ms: 0,
            contact_hold_ms: 0,
            inv_cells: 0.0,
            inv_torque_per_amp: 0.0,
            inv_temp_span: 0.0,
            grav_forearm: 0.0,
            grav_per_payload: 0.0,
            pos_integ_lim: 0.0,
        }
    }

    /// Настройка и запуск таймера: первое прерывание через один период.
    /// `retained_tw` — оценка обмотки, пережившая сброс (не после включения питания).
    pub fn init(
        &mut self,
        cfg: Config,
        from_block: bool,
        after_watchdog: bool,
        retained_tw: Option<f32>,
    ) {
        self.cfg = cfg;
        if let Some(tw) = retained_tw {
            self.tw = tw;
            self.tw_ready = true;
            self.tw_retained = true;
        }
        self.inv_rwh = 1.0 / cfg.rth_wh_k_per_w;
        // C_w = τ_w / R_wh → dt/C_w = dt·R_wh/τ_w.
        self.dt_over_cw = DT_OUTER * cfg.rth_wh_k_per_w / cfg.tau_w_s;
        self.stall_limit_ms = round_i32(cfg.stall_time_s * 1000.0).max(1) as u32;
        self.stall_ramp_total_ms = round_i32(cfg.stall_ramp_s * 1000.0).max(1) as u32;
        self.inv_stall_ramp_ms = 1.0 / self.stall_ramp_total_ms as f32;
        self.cfg_from_block = from_block;
        self.after_watchdog = after_watchdog;
        self.i_trip_q16 = (cfg.i_trip_a * 65536.0) as i32;
        self.kaw_q16 = (cfg.cur_kaw * DT_TICK * 65536.0) as i32;
        let vb = cfg.v_bus_nominal;
        self.kp_q16 = (cfg.cur_kp / vb * 65536.0) as i32;
        self.ki_q16 = (cfg.cur_ki * DT_TICK / vb * 65536.0) as i32;
        self.inv_cells = 1.0 / cfg.cells_series;
        self.inv_torque_per_amp = 1.0 / (cfg.kt_nm_per_a * cfg.gear_ratio * cfg.gear_efficiency);
        self.inv_temp_span = 1.0 / (cfg.temp_off_c - cfg.temp_derate_c);
        self.grav_forearm = cfg.forearm_mass_kg * G * cfg.forearm_com_m;
        self.grav_per_payload = G * cfg.hand_dist_m;
        self.pos_integ_lim = if cfg.pos_ki > 0.0 {
            cfg.i_max_a / cfg.pos_ki
        } else {
            0.0
        };
        self.next_cmp = timer::mtime() + MTIME_PER_TICK;
        timer::set_compare(self.next_cmp);
    }

    fn driver_off(&mut self) {
        gpio::set_low(PIN_EN);
        pwm::set_duty(PWM_MOTOR, 0);
        self.driver_on = false;
        self.integ_q16 = 0;
    }

    /// Тик 10 кГц (из прерывания таймера).
    pub fn fast_tick(&mut self) {
        self.next_cmp += MTIME_PER_TICK;
        timer::set_compare(self.next_cmp);
        self.tick = self.tick.wrapping_add(1);

        let levels = gpio::levels();
        let a = adc::read(ADC_ANGLE);
        let d = adc::read(ADC_CURRENT) as i32 - I_ZERO_CODE;
        let i_q16 = d * I_Q16_PER_CODE;
        self.angle_sum += a;
        self.cur_sum += i_q16;
        self.i2_sum += (d * d) as u32;
        self.n_sum += 1;

        // Мгновенные защиты: аварийная кнопка (активный низкий) и обрыв/КЗ датчика угла.
        if levels & (1 << PIN_ESTOP) == 0 {
            self.latched |= F_ESTOP;
        }
        if !(SENSOR_LO_CODE..=SENSOR_HI_CODE).contains(&a) {
            self.latched |= F_SENSOR;
        }
        // Перегрузка по току: |I| > i_trip дольше 2 мс.
        if i_q16.unsigned_abs() > self.i_trip_q16 as u32 {
            self.oc_count += 1;
            if self.oc_count > OC_TICKS {
                self.latched |= F_OVERCURRENT;
            }
        } else {
            self.oc_count = 0;
        }

        if self.mode != 0 && self.latched == 0 && self.latched2 == 0 && !self.blocked {
            if !self.driver_on {
                gpio::set_high(PIN_EN);
                self.driver_on = true;
            }
            // ПИ по току с обратным расчётом антинасыщения (back-calculation).
            let err = self.i_ref_q16 - i_q16;
            let p = ((self.kp_q16 as i64 * err as i64) >> 16) as i32;
            let u_unsat = p.saturating_add(self.integ_q16);
            let u = u_unsat.clamp(-DUTY_MAX_Q16, DUTY_MAX_Q16);
            let di = ((self.ki_q16 as i64 * err as i64) >> 16) as i32
                + ((self.kaw_q16 as i64 * (u - u_unsat) as i64) >> 16) as i32;
            self.integ_q16 = (self.integ_q16 + di).clamp(-65_536, 65_536);
            let flex = u >= 0;
            if flex != self.dir_flex {
                gpio::write(PIN_DIR, flex);
                self.dir_flex = flex;
            }
            pwm::set_duty(PWM_MOTOR, u.unsigned_abs());
        } else if self.driver_on {
            self.driver_off();
        } else {
            self.integ_q16 = 0;
        }

        if self.tick.is_multiple_of(10) {
            self.outer();
        }
    }

    /// Внешний контур 1 кГц. Только умножения (деления заменены заранее
    /// посчитанными обратными величинами) — программная плавающая точка RV32IMC.
    fn outer(&mut self) {
        let angle = if self.n_sum == 10 {
            self.angle_sum as f32 * DEG_PER_CODE_SUM10 - DEG_OFFSET
        } else {
            let n = self.n_sum.max(1) as f32;
            (self.angle_sum as f32 / n) * (3.3 / 4096.0 * (150.0 / 2.7)) - DEG_OFFSET
        };
        let inv_n = if self.n_sum == 10 {
            0.1
        } else {
            1.0 / self.n_sum.max(1) as f32
        };
        let cur = self.cur_sum as f32 * (1.0 / 65536.0) * inv_n;
        let i2_mean = self.i2_sum as f32 * I2_PER_CODE2 * inv_n;
        self.angle_sum = 0;
        self.cur_sum = 0;
        self.i2_sum = 0;
        self.n_sum = 0;
        if self.first_outer {
            self.angle_deg = angle;
            self.first_outer = false;
        }
        let raw_vel = (angle - self.angle_deg) * (1.0 / DT_OUTER);
        self.vel_dps += VEL_ALPHA * (raw_vel - self.vel_dps);
        self.stall_vel += STALL_VEL_ALPHA * (raw_vel - self.stall_vel);
        self.angle_deg = angle;
        self.current_a = cur;

        let c = self.cfg;
        let mut live = 0u8;

        // Температура: линейное снижение i_max от temp_derate до temp_off, выше — выкл.
        let temp = (adc::read(ADC_TEMP) as f32 * (3.3 / 4096.0) - 0.5) * 100.0;
        self.temp_c = temp;
        let k_t = if temp >= c.temp_off_c {
            live |= F_OVERTEMP;
            0.0
        } else if temp > c.temp_derate_c {
            (c.temp_off_c - temp) * self.inv_temp_span
        } else {
            1.0
        };
        self.derating = temp > c.temp_derate_c && temp < c.temp_off_c;

        // Обмотка: оценка по I²·R(T) и измеренному корпусу.
        let mut live2 = 0u8;
        let tw = self.winding_step(i2_mean, temp);
        let t_max = c.t_winding_max_c;
        let k_w = if tw >= t_max {
            live2 |= F2_WINDING_HOT;
            0.0
        } else if tw > t_max - WINDING_DERATE_BAND_C {
            live2 |= F2_WINDING_HOT;
            (t_max - tw) * (1.0 / WINDING_DERATE_BAND_C)
        } else {
            1.0
        };
        self.blocked = temp >= c.temp_off_c || tw >= t_max;

        // Батарея.
        let vbat = adc::read(ADC_VBAT) as f32 * (3.3 / 4096.0 * 11.0);
        self.vbat_v = vbat;
        let per_cell = vbat * self.inv_cells;
        if per_cell < BATT_LOW_V_CELL {
            self.batt_low_count += 1;
            if self.batt_low_count >= BATT_LOW_MS {
                self.latched |= F_BATT_LOW;
            }
        } else {
            self.batt_low_count = 0;
        }
        if per_cell < BATT_WARN_V_CELL {
            live |= F_BATT_WARN;
        }

        // Связь: после 500 мс тишины — спад поддержки до нуля за 200 мс.
        self.ms_since_cmd = self.ms_since_cmd.saturating_add(1);
        let mut ramp = 1.0f32;
        if self.mode != 0 && self.ms_since_cmd > CMD_TIMEOUT_MS {
            live |= F_CMD_TIMEOUT;
            let over = (self.ms_since_cmd - CMD_TIMEOUT_MS) as f32;
            ramp = (1.0 - over * (1.0 / RAMP_MS)).max(0.0);
        }

        // Задание тока.
        let faulted = self.latched != 0 || self.blocked;
        let pos_target = if self.contact_hold {
            self.contact_target
        } else {
            self.target_deg
        };
        let pos_err = pos_target - angle;
        let gravity_amps = (self.grav_forearm + self.payload_kg * self.grav_per_payload)
            * sin_deg(angle)
            * self.inv_torque_per_amp;
        let mut i_cmd = if self.mode == 0 {
            self.ref_valid = false;
            0.0
        } else if self.mode == 1 {
            self.ref_valid = false;
            self.assist_pct as f32 * 0.01 * gravity_amps
        } else {
            if !self.ref_valid || faulted {
                self.ref_deg = angle;
                self.ref_vel = 0.0;
                self.ref_valid = true;
            }
            let (ref_deg, ref_vel) = self.ref_step(pos_target);
            let e = ref_deg - angle;
            if faulted {
                self.pos_integ = 0.0;
            } else if e > -POS_INTEG_BAND_DEG && e < POS_INTEG_BAND_DEG {
                self.pos_integ =
                    (self.pos_integ + e * DT_OUTER).clamp(-self.pos_integ_lim, self.pos_integ_lim);
            }
            c.pos_kp * e + c.pos_ki * self.pos_integ + c.pos_kd * (ref_vel - self.vel_dps)
                + gravity_amps
        };
        i_cmd *= ramp;
        let i_lim = c.i_max_a * k_t.min(k_w);
        i_cmd = i_cmd.clamp(-i_lim, i_lim);

        // Контакт: ток заметно выше, чем нужно на вес руки и груза. Рука отводится назад.
        if self.contact_hold {
            live2 |= F2_CONTACT;
        }
        if self.mode == 2 && c.contact_torque_nm > 0.0 && !faulted && self.driver_on {
            if self.contact_hold {
                self.contact_hold_ms = self.contact_hold_ms.saturating_add(1);
                if self.contact_hold_ms >= CONTACT_RETREAT_MS
                    && fabs(angle - self.contact_target) > CONTACT_RETREAT_TOL_DEG
                    && !self.stall_ramping
                    && self.latched2 & F2_STALL == 0
                {
                    self.stall_ramping = true;
                    self.stall_ramp_ms = 0;
                }
            } else if fabs(cur - gravity_amps)
                > c.contact_torque_nm * self.inv_torque_per_amp
                    + (1.0 - c.gear_efficiency * c.gear_efficiency) * fabs(gravity_amps)
                && fabs(self.vel_dps) <= 0.5 * fabs(self.ref_vel) + CONTACT_STILL_DPS
            {
                // Порог учитывает потери редуктора под нагрузкой (при опускании ток заметно
                // ниже модели веса), а контактом считается только отставание от траектории.
                self.contact_ms += 1;
                if self.contact_ms >= CONTACT_CONFIRM_MS {
                    let back = if cur > gravity_amps {
                        -CONTACT_BACKOFF_DEG
                    } else {
                        CONTACT_BACKOFF_DEG
                    };
                    self.contact_target = (angle + back).clamp(c.angle_min_deg, c.angle_max_deg);
                    self.contact_hold = true;
                    self.contact_hold_ms = 0;
                    self.contact_ms = 0;
                    self.ref_deg = angle;
                    self.ref_vel = 0.0;
                    self.pos_integ = 0.0;
                    live2 |= F2_CONTACT;
                }
            } else {
                self.contact_ms = 0;
            }
        }

        // Заклинивание: требуется движение (режим 2, цель далеко), ток у действующего
        // предела, сустав стоит. Срабатывание — не мгновенный обрыв, а спад тока до
        // нуля за stall_ramp_s (груз не падает рывком), затем выключение и защёлка.
        let mut stall_scale = 1.0f32;
        if self.stall_ramping {
            self.stall_ramp_ms += 1;
            live2 |= F2_STALL;
            if self.stall_ramp_ms >= self.stall_ramp_total_ms {
                self.stall_ramping = false;
                self.latched2 |= F2_STALL;
                stall_scale = 0.0;
            } else {
                stall_scale = 1.0 - self.stall_ramp_ms as f32 * self.inv_stall_ramp_ms;
            }
        } else if self.latched2 & F2_STALL == 0 {
            let demand = self.mode == 2 && fabs(pos_err) > c.stall_pos_err_deg;
            let high = self.driver_on && i_lim > 0.1 && fabs(cur) >= STALL_CURRENT_FRAC * i_lim;
            if demand && high && fabs(self.stall_vel) < STALL_SPEED_DPS {
                self.stall_ms += 1;
                if self.stall_ms >= self.stall_limit_ms {
                    self.stall_ramping = true;
                    self.stall_ramp_ms = 0;
                    self.stall_ms = 0;
                    live2 |= F2_STALL;
                }
            } else {
                self.stall_ms = 0;
            }
        }
        i_cmd *= stall_scale;
        // Мягкие пределы угла: выше максимума — без момента сгибания, ниже минимума — без разгибания.
        if angle >= c.angle_max_deg && i_cmd > 0.0 {
            i_cmd = 0.0;
            live |= F_ANGLE_LIMIT;
        }
        if angle <= c.angle_min_deg && i_cmd < 0.0 {
            i_cmd = 0.0;
            live |= F_ANGLE_LIMIT;
        }
        self.i_ref_q16 = (i_cmd * 65536.0) as i32;

        // Усиления токового контура в долях заполнения — с поправкой на напряжение
        // шины (одно деление).
        let inv_vb = 1.0 / vbat.clamp(0.5 * c.v_bus_nominal, 1.5 * c.v_bus_nominal);
        self.kp_q16 = (c.cur_kp * inv_vb * 65536.0) as i32;
        self.ki_q16 = (c.cur_ki * (DT_TICK * 65536.0) * inv_vb) as i32;

        self.live = live;
        self.seen |= live | self.latched;
        self.live2 = live2;
        self.seen2 |= live2 | self.latched2;

        // SAFETY: запись блока целиком; читает только хост (как отладчик).
        unsafe {
            (&raw mut EXO_DIAG).write_volatile(Diag {
                magic: 0x4741_4944,
                winding_est_c: tw,
                housing_c: temp,
                i2_mean_a2: i2_mean,
                i_limit_a: i_lim,
                stall_ms: self.stall_ms,
                stall_vel_dps: self.stall_vel,
                winding_retained: self.tw_retained as u32,
                stall_state: if self.stall_ramping {
                    1
                } else if self.latched2 & F2_STALL != 0 {
                    2
                } else {
                    0
                },
            });
        }
    }

    /// Шаг траектории 1 кГц: опорный угол идёт к цели не быстрее `speed_limit_dps`, с
    /// ограниченным ускорением и плавным подходом. Возвращает (угол, скорость).
    fn ref_step(&mut self, target: f32) -> (f32, f32) {
        let limit = self.cfg.speed_limit_dps;
        if limit <= 0.0 {
            self.ref_deg = target;
            self.ref_vel = 0.0;
            return (target, 0.0);
        }
        let want = ((target - self.ref_deg) * REF_GAIN_PER_S).clamp(-limit, limit);
        let dv = REF_ACCEL_DPS2 * DT_OUTER;
        self.ref_vel += (want - self.ref_vel).clamp(-dv, dv);
        self.ref_deg += self.ref_vel * DT_OUTER;
        (self.ref_deg, self.ref_vel)
    }

    /// Шаг 1 мс тепловой модели обмотки (явный Эйлер, τ_w ≫ 1 мс):
    /// C_w·dT_w/dt = I²·R_ref·(1 + α·(T_w − T_ref)) − (T_w − T_корпуса)/R_wh.
    /// Суммирование с компенсацией (Кэхэн): приращение за 1 мс ~10⁻³ К при T ~10² °C —
    /// без компенсации округление f32 копило бы смещение. Возвращает оценку, °C.
    pub fn winding_step(&mut self, i2_mean: f32, housing_c: f32) -> f32 {
        let c = &self.cfg;
        if !self.tw_ready {
            // После включения питания считаем обмотку в равновесии с корпусом.
            self.tw = housing_c;
            self.tw_comp = 0.0;
            self.tw_ready = true;
        }
        let r = c.motor_r_ohm * (1.0 + ALPHA_CU * (self.tw - c.motor_r_ref_c));
        let inc = (i2_mean * r - (self.tw - housing_c) * self.inv_rwh) * self.dt_over_cw;
        let y = inc - self.tw_comp;
        let t = self.tw + y;
        self.tw_comp = (t - self.tw) - y;
        self.tw = t;
        keep_store(t);
        t
    }

    /// Команда с телефона.
    pub fn apply_command(&mut self, mode: u8, assist: u8, target10: u16, payload10: u16, seq: u8) {
        if mode > 2 {
            return;
        }
        self.ms_since_cmd = 0;
        self.last_cmd_seq = seq;
        self.assist_pct = assist.min(100);
        self.target_deg = (target10 as f32 * 0.1).clamp(0.0, 150.0);
        self.payload_kg = (payload10 as f32 * 0.1).min(20.0);
        if mode == 0 {
            // Режим 0 снимает защёлки; если причина осталась — защёлкнется снова.
            self.latched = 0;
            self.latched2 = 0;
            self.oc_count = 0;
            self.batt_low_count = 0;
            self.stall_ms = 0;
            self.stall_ramping = false;
            self.contact_hold = false;
            self.contact_ms = 0;
            self.contact_hold_ms = 0;
        }
        if mode != self.mode {
            self.pos_integ = 0.0;
        }
        self.mode = mode;
    }

    /// Есть ли неисправность, выключающая драйвер (для индикатора).
    pub fn any_fault(&self) -> bool {
        self.latched != 0 || self.latched2 != 0 || self.blocked || self.contact_hold
    }

    /// Пакет телеметрии (14 байт), формат — в `MEMORY_MAP.md`.
    pub fn telemetry(&mut self, seq: u8) -> [u8; 14] {
        let state = (self.mode & 3)
            | (self.driver_on as u8) << 2
            | (self.derating as u8) << 3
            | (self.after_watchdog as u8) << 4
            | (!self.cfg_from_block as u8) << 5
            | ((self.ms_since_cmd <= CMD_TIMEOUT_MS) as u8) << 6;
        let flags = self.seen | self.latched | self.live;
        let flags2 = self.seen2 | self.latched2 | self.live2;
        self.seen = 0;
        self.seen2 = 0;
        let angle10 = round_i32(self.angle_deg * 10.0).clamp(0, 65_535) as u16;
        let cur100 = round_i32(self.current_a * 100.0).clamp(-32_768, 32_767) as i16;
        let temp10 = round_i32(self.temp_c * 10.0).clamp(-32_768, 32_767) as i16;
        let vbat100 = round_i32(self.vbat_v * 100.0).clamp(0, 65_535) as u16;
        let mut p = [0u8; 14];
        p[0] = 0x7e;
        p[1] = state;
        p[2..4].copy_from_slice(&angle10.to_le_bytes());
        p[4..6].copy_from_slice(&cur100.to_le_bytes());
        p[6..8].copy_from_slice(&temp10.to_le_bytes());
        p[8..10].copy_from_slice(&vbat100.to_le_bytes());
        p[10] = flags;
        p[11] = seq;
        p[12] = self.last_cmd_seq;
        p[13] = flags2;
        p
    }
}
