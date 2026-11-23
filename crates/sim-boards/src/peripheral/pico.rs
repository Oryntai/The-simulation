//! RP2040 Datasheet: §2.14 RESETS, §2.19 GPIO, §4.1 ADC, §4.5 PWM,
//! §4.6 TIMER, §4.7 UART (PL011), §4.8 WATCHDOG.

use crate::clock::TickAcc;

use super::{Access, Adc, Pins, Uart, checked_rw, command, invalid, ro, rw, unsupported};

const GPIO_MASK: u32 = (1 << 30) - 1;
const SYS_MHZ: u64 = 125; // Профиль clk_sys Pico SDK.
const RESET_MASK: u32 = (1 << 25) - 1; // RP2040 RESETS_RESET.
const MASK_12: u32 = (1 << 12) - 1;
const XOSC_MHZ: u64 = 12; // Схема Pico.
const PSM_WDSEL_ALL_EXCEPT_OSC: u32 = 0x1fffc; // Pico SDK watchdog.c / PSM_WDSEL.
const PSM_WDSEL_MASK: u32 = 0x1ffff;

#[derive(Clone)]
struct Pwm {
    csr: u32,
    div: u32,
    ctr: u32,
    cc: u32,
    top: u32,
    active_cc: u32,
    active_top: u32,
    frac: u64,
}

impl Default for Pwm {
    fn default() -> Self {
        Self {
            csr: 0,
            div: 0x10,
            ctr: 0,
            cc: 0,
            top: 0xffff,
            active_cc: 0,
            active_top: 0xffff,
            frac: 0,
        }
    }
}

impl Pwm {
    fn tick(&mut self, cycles: u64) -> bool {
        if self.csr & 1 == 0 {
            return false;
        }
        let tally = self.frac + cycles * 16;
        self.frac = tally % u64::from(self.div);
        let ticks = (tally / u64::from(self.div)) as u32;
        let end = self.ctr + ticks;
        let wrap = end > self.active_top;
        if wrap {
            let excess = end.saturating_sub(self.active_top + 1);
            self.active_cc = self.cc;
            self.active_top = self.top;
            self.ctr = excess % (self.active_top + 1);
        } else {
            self.ctr = end;
        }
        wrap
    }

    fn compare(&self, chan: u32) -> u32 {
        self.active_cc >> (chan * 16) & 0xffff
    }
    fn invert(&self, chan: u32) -> bool {
        self.csr & (4 << chan) != 0
    }
}

#[derive(Clone)]
struct Pl011 {
    integer: u32,
    fraction: u32,
    line: u32,
    control: u32,
}

impl Default for Pl011 {
    fn default() -> Self {
        Self {
            integer: 0,
            fraction: 0,
            line: 0,
            control: 0x300,
        }
    }
}

#[derive(Clone)]
pub(super) struct Pico {
    pub pins: Pins,
    mux: [u32; 30],
    pads: [u32; 30],
    reset: u32,
    timer_clock: TickAcc,
    timer: u64,
    timer_latch: u32,
    timer_write: u32,
    alarm: [u32; 4],
    armed: u32,
    timer_raw: u32,
    timer_enable: u32,
    timer_force: u32,
    paused: u32,
    pwm: [Pwm; 8],
    pwm_raw: u32,
    pwm_irq: u32,
    pwm_force: u32,
    adc_cs: u32,
    serial: [Pl011; 2],
    wdt_ctrl: u32,
    wdt_psm: u32,
    prepared_wdt_cycles: Option<u64>,
    pub forced_reset: bool,
    wdt_tick: u32,
    deadline: Option<u64>,
    pub watchdog_reason: u32,
    scratch: [u32; 8],
}

impl Default for Pico {
    fn default() -> Self {
        Self {
            pins: Pins::default(),
            mux: [31; 30],
            pads: [0x56; 30],
            reset: 0,
            timer_clock: TickAcc::default(),
            timer: 0,
            timer_latch: 0,
            timer_write: 0,
            alarm: [0; 4],
            armed: 0,
            timer_raw: 0,
            timer_enable: 0,
            timer_force: 0,
            paused: 0,
            pwm: std::array::from_fn(|_| Pwm::default()),
            pwm_raw: 0,
            pwm_irq: 0,
            pwm_force: 0,
            adc_cs: 0,
            serial: std::array::from_fn(|_| Pl011::default()),
            wdt_ctrl: 0,
            wdt_tick: 0x20c,
            wdt_psm: 0,
            prepared_wdt_cycles: None,
            forced_reset: false,
            deadline: None,
            watchdog_reason: 0,
            scratch: [0; 8],
        }
    }
}

impl Pico {
    pub fn retain_watchdog_scratch(&mut self, old: &Self) {
        self.scratch = old.scratch;
    }

    pub fn tick(&mut self, passed: u64, now: u64) -> bool {
        if self.paused == 0 && self.reset & (1 << 21) == 0 && self.wdt_tick & 0x200 != 0 {
            // Опорная частота XOSC/clk_ref 12 МГц (схема Pico).
            let divider = u64::from(self.wdt_tick & 0x1ff) * SYS_MHZ;
            if divider != 0 {
                let ticks = self.timer_clock.ticks(passed, XOSC_MHZ, divider);
                if ticks != 0 {
                    let old = self.timer as u32;
                    self.timer = self.timer.wrapping_add(ticks);
                    for (i, &alarm) in self.alarm.iter().enumerate() {
                        let distance = alarm.wrapping_sub(old);
                        if self.armed & (1 << i) != 0
                            && distance != 0
                            && u64::from(distance) <= ticks
                        {
                            self.armed &= !(1 << i);
                            self.timer_raw |= 1 << i;
                        }
                    }
                }
            }
        }
        if self.reset & (1 << 14) == 0 {
            for (i, pwm) in self.pwm.iter_mut().enumerate() {
                if pwm.tick(passed) {
                    self.pwm_raw |= 1 << i;
                }
            }
        }
        self.deadline.is_some_and(|d| now >= d)
    }

    pub fn interrupts(&self) -> u64 {
        u64::from((self.timer_raw & self.timer_enable) | self.timer_force)
            | (u64::from((self.pwm_raw & self.pwm_irq) | self.pwm_force != 0) << 4)
    }

    pub fn output(&self, pin: u32) -> Option<bool> {
        let mux = *self.mux.get(pin as usize)?;
        if self.pads[pin as usize] & 0x80 != 0 || self.reset & (1 << 5) != 0 {
            return None;
        }
        match mux {
            5 => self.pins.output(pin),
            4 => {
                self.pwm(pin)?;
                let p = &self.pwm[((pin / 2) % 8) as usize];
                Some((p.ctr < p.compare(pin % 2)) ^ p.invert(pin % 2))
            }
            _ => None,
        }
    }

    pub fn pwm_period(&self, pin: u32) -> Option<f64> {
        self.pwm(pin)?;
        let p = &self.pwm[((pin / 2) % 8) as usize];
        // RP2040 §4.5: период счёта (TOP+1) и делитель 8.4.
        Some(f64::from((p.active_top + 1) * p.div) / (16.0 * (SYS_MHZ * 1_000_000) as f64))
    }

    pub fn pwm(&self, pin: u32) -> Option<f64> {
        if *self.mux.get(pin as usize)? != 4
            || self.pads[pin as usize] & 0x80 != 0
            || self.reset & ((1 << 5) | (1 << 14)) != 0
        {
            return None;
        }
        let p = &self.pwm[((pin / 2) % 8) as usize];
        let period = p.active_top + 1;
        let high = p.compare(pin % 2).min(period);
        (p.csr & 1 != 0).then_some(
            f64::from(if p.invert(pin % 2) {
                period - high
            } else {
                high
            }) / f64::from(period),
        )
    }

    pub fn access(
        &mut self,
        addr: u32,
        v: Option<u32>,
        now: u64,
        uart: &mut Uart,
        adc: &mut Adc,
    ) -> Access {
        if (0xd000_0000..0xd000_0100).contains(&addr) {
            return match addr & 0xff {
                0 => ro(v, 0),
                4 => ro(
                    v,
                    (0..30).fold(0, |bits, i| {
                        let level = if self.pads[i] & 0x40 != 0 {
                            let pull_up = if self.pads[i] & 0xc == 8 { 1 << i } else { 0 };
                            let read = if self.mux[i] == 5 && self.pads[i] & 0x80 == 0 {
                                self.pins.read(pull_up)
                            } else {
                                self.pins.input_with_pull(pull_up)
                            };
                            read & (1 << i)
                        } else {
                            0
                        };
                        bits | level
                    }),
                ),
                0x10 => rw(v, &mut self.pins.out, GPIO_MASK),
                0x14 => command(v, |v| self.pins.out |= v & GPIO_MASK),
                0x18 => command(v, |v| self.pins.out &= !v),
                0x1c => command(v, |v| self.pins.out ^= v & GPIO_MASK),
                0x20 => rw(v, &mut self.pins.enable, GPIO_MASK),
                0x24 => command(v, |v| self.pins.enable |= v & GPIO_MASK),
                0x28 => command(v, |v| self.pins.enable &= !v),
                0x2c => command(v, |v| self.pins.enable ^= v & GPIO_MASK),
                _ => Ok(None),
            };
        }
        let base = addr & !0x3fff;
        let offset = addr & 0xfff;
        let alias = addr >> 12 & 3;
        let reset_bit = match base {
            0x4001_4000 => Some(5),
            0x4001_c000 => Some(8),
            0x4003_4000 => Some(22),
            0x4003_8000 => Some(23),
            0x4004_c000 => Some(0),
            0x4005_0000 => Some(14),
            0x4005_4000 => Some(21),
            _ => None,
        };
        if reset_bit.is_some_and(|bit| self.reset & (1 << bit) != 0) {
            return Err(invalid("доступ к блоку RP2040, удерживаемому в сбросе"));
        }
        if alias == 0 {
            return self.reg(base, offset, v, now, uart, adc);
        }
        // RP2040 §2.1.2: атомарные псевдонимы APB. Только для регистров без побочных эффектов чтения.
        let safe = matches!(
            (base, offset),
            (0x4000_c000, 0)
                | (0x4001_0000, 8)
                | (0x4005_4000, 0x38 | 0x3c)
                | (0x4005_0000, 0xa0 | 0xa8 | 0xac)
                | (0x4005_8000, 0x2c)
        ) || base == 0x4001_4000 && offset < 30 * 8 && offset % 8 == 4
            || base == 0x4001_c000 && (4..=120).contains(&offset);
        if !safe {
            return Ok(None);
        }
        let Some(v) = v else {
            return self.reg(base, offset, None, now, uart, adc);
        };
        let Some(old) = self.reg(base, offset, None, now, uart, adc)? else {
            return Ok(None);
        };
        let val = match alias {
            1 => old ^ v,
            2 => old | v,
            _ => old & !v,
        };
        self.reg(base, offset, Some(val), now, uart, adc)
    }

    fn reg(
        &mut self,
        base: u32,
        o: u32,
        v: Option<u32>,
        now: u64,
        uart: &mut Uart,
        adc: &mut Adc,
    ) -> Access {
        match base {
            0x4000_c000 => match o {
                0 => {
                    if v.is_some_and(|v| v & !RESET_MASK != 0 || v & !self.reset != 0) {
                        return Err(unsupported(
                            "RP2040 RESETS: повторное удержание блока в сбросе",
                        ));
                    }
                    rw(v, &mut self.reset, RESET_MASK)
                }
                8 => ro(v, RESET_MASK & !self.reset),
                _ => Ok(None),
            },
            0x4001_0000 if o == 8 => checked_rw(v, &mut self.wdt_psm, PSM_WDSEL_MASK, "PSM WDSEL"),
            0x4001_4000 if o < 30 * 8 && o % 8 == 4 => checked_rw(
                v,
                &mut self.mux[(o / 8) as usize],
                31,
                "GPIO CTRL (переопределение сигнала)",
            ),
            0x4001_c000 if (4..=120).contains(&o) => {
                rw(v, &mut self.pads[(o / 4 - 1) as usize], 0xff)
            }
            0x4005_4000 => match o {
                0 => command(v, |v| {
                    self.timer = (u64::from(v) << 32) | u64::from(self.timer_write)
                }),
                4 => command(v, |v| self.timer_write = v),
                8 => ro(v, self.timer_latch),
                12 if v.is_none() => {
                    self.timer_latch = (self.timer >> 32) as u32;
                    ro(v, self.timer as u32)
                }
                0x10..=0x1c => {
                    if v.is_some() {
                        self.armed |= 1 << ((o - 0x10) / 4);
                    }
                    rw(v, &mut self.alarm[((o - 0x10) / 4) as usize], u32::MAX)
                }
                0x20 => {
                    if let Some(value) = v {
                        self.armed &= !value;
                        Ok(Some(0))
                    } else {
                        ro(v, self.armed)
                    }
                }
                0x24 => ro(v, (self.timer >> 32) as u32),
                0x28 => ro(v, self.timer as u32),
                0x30 => rw(v, &mut self.paused, 1),
                0x34 => {
                    if let Some(v) = v {
                        self.timer_raw &= !v;
                        Ok(Some(0))
                    } else {
                        ro(v, self.timer_raw)
                    }
                }
                0x38 => rw(v, &mut self.timer_enable, 15),
                0x3c => rw(v, &mut self.timer_force, 15),
                0x40 => ro(v, self.timer_raw & self.timer_enable | self.timer_force),
                _ => Ok(None),
            },
            0x4005_0000 => {
                if o < 0xa0 {
                    let pwm = &mut self.pwm[(o / 0x14) as usize];
                    return match o % 0x14 {
                        0 => checked_rw(
                            v,
                            &mut pwm.csr,
                            0x0d,
                            "RP2040 PWM CSR (режим кроме свободного счёта)",
                        ),
                        4 => {
                            if v.is_some_and(|x| !(0x10..=MASK_12).contains(&x)) {
                                return Err(invalid("недопустимый делитель PWM"));
                            }
                            pwm.frac = if v.is_some() { 0 } else { pwm.frac };
                            rw(v, &mut pwm.div, MASK_12)
                        }
                        8 => rw(v, &mut pwm.ctr, 0xffff),
                        12 => {
                            if let Some(v) = v.filter(|_| pwm.csr & 1 == 0) {
                                pwm.active_cc = v;
                            }
                            rw(v, &mut pwm.cc, u32::MAX)
                        }
                        16 => {
                            if let Some(v) = v.filter(|_| pwm.csr & 1 == 0) {
                                pwm.active_top = v & 0xffff;
                            }
                            rw(v, &mut pwm.top, 0xffff)
                        }
                        _ => Ok(None),
                    };
                }
                match o {
                    0xa0 => {
                        if let Some(v) = v {
                            for (i, p) in self.pwm.iter_mut().enumerate() {
                                p.csr = (p.csr & !1) | ((v >> i) & 1);
                            }
                            Ok(Some(0))
                        } else {
                            ro(
                                v,
                                self.pwm
                                    .iter()
                                    .enumerate()
                                    .fold(0, |bits, (i, p)| bits | ((p.csr & 1) << i)),
                            )
                        }
                    }
                    0xa4 => {
                        if let Some(v) = v {
                            self.pwm_raw &= !v;
                            Ok(Some(0))
                        } else {
                            ro(v, self.pwm_raw)
                        }
                    }
                    0xa8 => rw(v, &mut self.pwm_irq, 255),
                    0xac => rw(v, &mut self.pwm_force, 255),
                    0xb0 => ro(v, self.pwm_raw & self.pwm_irq | self.pwm_force),
                    _ => Ok(None),
                }
            }
            0x4004_c000 => match o {
                0 => {
                    if let Some(v) = v {
                        if v & !0x0000_7007 != 0 {
                            return Err(unsupported("RP2040 ADC, только одиночное преобразование"));
                        }
                        self.adc_cs = v & 0x7003;
                        if v & 4 != 0 {
                            if v & 1 == 0 || adc.deadline.is_some() || (v >> 12 & 7) > 3 {
                                return Err(invalid(
                                    "АЦП выключен, занят или выбран немоделируемый канал",
                                ));
                            }
                            // §4.1: 96 clk_adc, при clk_adc 48 МГц и clk_sys 125 МГц.
                            let ch = (v >> 12 & 7) as usize;
                            if self.pads[26 + ch] & 0xcc != 0x80 {
                                return Err(invalid(
                                    "включите аналоговый режим площадки RP2040: OD=1, IE=PUE=PDE=0",
                                ));
                            }
                            adc.start(ch, 3.3, 12, now, 96 * SYS_MHZ / 48)?;
                        }
                        Ok(Some(0))
                    } else {
                        ro(
                            v,
                            self.adc_cs
                                | (u32::from(self.adc_cs & 1 != 0 && adc.deadline.is_none()) << 8),
                        )
                    }
                }
                4 if adc.ready => ro(v, adc.result),
                4 => Err(invalid("результат АЦП ещё не получен")),
                _ => Ok(None),
            },
            0x4003_4000 | 0x4003_8000 => {
                let instance = usize::from(base == 0x4003_8000);
                let port = &mut self.serial[instance];
                let tx = &mut uart.ports[instance];
                match o {
                    0 => {
                        let Some(byte) = v else {
                            return Ok(None);
                        };
                        let function_pins = if instance == 0 {
                            [0, 12, 16, 28]
                        } else {
                            [4, 8, 20, 24]
                        };
                        if port.control & 0x101 != 0x101
                            || port.line & !0x10 != 0x60
                            || !function_pins
                                .into_iter()
                                .any(|pin| self.mux[pin] == 2 && self.pads[pin] & 0x80 == 0)
                        {
                            return Err(invalid(
                                "PL011 требует режим 8N1, включённый TX и маршрутизацию вывода",
                            ));
                        }
                        // PL011: 16 × (IBRD + FBRD/64), clk_peri = clk_sys в профиле Pico.
                        let div64 = u64::from(port.integer * 64 + port.fraction);
                        let capacity = if port.line & 0x10 != 0 { 32 } else { 1 }; // RP2040 §4.7, PL011.
                        tx.start(now, (10 * div64).div_ceil(4), byte, capacity)
                    }
                    0x18 => {
                        let capacity = if port.line & 0x10 != 0 { 32 } else { 1 };
                        ro(
                            v,
                            0x10 | (u32::from(tx.busy()) << 3)
                                | (u32::from(tx.queued() >= capacity) << 5)
                                | (u32::from(tx.queued() == 0) << 7),
                        )
                    }
                    0x24 => rw(v, &mut port.integer, 0xffff),
                    0x28 => rw(v, &mut port.fraction, 63),
                    0x2c => checked_rw(v, &mut port.line, 0x70, "PL011 LCR_H"),
                    0x30 => checked_rw(v, &mut port.control, 0x301, "PL011 CR"),
                    _ => Ok(None),
                }
            }
            0x4005_8000 => match o {
                0 => {
                    if let Some(v) = v {
                        if v & !0xc700_0000 != 0 {
                            return Err(unsupported("RP2040 WATCHDOG CTRL"));
                        }
                        if v >> 30 != 0 && self.wdt_psm != PSM_WDSEL_ALL_EXCEPT_OSC {
                            return Err(unsupported(
                                "RP2040 WATCHDOG: требуется профиль PSM WDSEL pico-sdk",
                            ));
                        }
                        if v & (1 << 31) != 0 {
                            self.forced_reset = true;
                            self.deadline = Some(now);
                        } else if v & (1 << 30) == 0 {
                            if let Some(d) = self.deadline.take() {
                                self.prepared_wdt_cycles = Some(d.saturating_sub(now));
                            }
                        } else if self.wdt_ctrl & (1 << 30) == 0 {
                            let duration = self.prepared_wdt_cycles.ok_or_else(|| {
                                invalid("перед включением WATCHDOG требуется записать LOAD")
                            })?;
                            self.deadline = Some(now + duration);
                        }
                        self.wdt_ctrl = v & 0x4700_0000;
                        Ok(Some(0))
                    } else {
                        let remaining = self
                            .deadline
                            .map_or(self.prepared_wdt_cycles.unwrap_or(0), |d| {
                                d.saturating_sub(now)
                            });
                        let div = u64::from(self.wdt_tick & 0x1ff) * SYS_MHZ;
                        let time = (remaining * XOSC_MHZ * 2).checked_div(div).unwrap_or(0) as u32;
                        Ok(Some(self.wdt_ctrl | (time & 0xff_ffff)))
                    }
                }
                4 if v.is_some() => {
                    let v = v.unwrap() & 0xff_ffff;
                    let ticks = u64::from(self.wdt_tick & 0x1ff);
                    if self.wdt_tick & 0x200 == 0 || ticks == 0 {
                        return Err(invalid("не настроен такт WATCHDOG"));
                    }
                    // §4.8 erratum RP2040-E1: счётчик убывает на два за тик.
                    let delay = TickAcc::default().cycles_for(
                        u64::from(v).div_ceil(2),
                        XOSC_MHZ,
                        ticks * SYS_MHZ,
                    );
                    self.prepared_wdt_cycles = Some(delay);
                    if self.wdt_ctrl & (1 << 30) != 0 {
                        self.deadline = Some(now + delay);
                    }
                    Ok(Some(0))
                }
                8 => ro(v, self.watchdog_reason),
                0xc..=0x28 => rw(v, &mut self.scratch[((o - 0xc) / 4) as usize], u32::MAX),
                0x2c => {
                    if let Some(v) = v {
                        if self.wdt_ctrl & (1 << 30) != 0 && v != self.wdt_tick
                            || v & 0x200 != 0 && v & 0x1ff == 0
                        {
                            return Err(unsupported(
                                "изменение частоты работающего WATCHDOG или нулевой делитель",
                            ));
                        }
                        if self.wdt_tick != v & 0x3ff {
                            self.timer_clock.clear();
                            self.prepared_wdt_cycles = None;
                        }
                        self.wdt_tick = v & 0x3ff;
                        Ok(Some(0))
                    } else {
                        ro(v, self.wdt_tick | ((self.wdt_tick & 0x200) << 1))
                    }
                }
                _ => Ok(None),
            },
            _ => Ok(None),
        }
    }
}
