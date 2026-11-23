//! RM0008: RCC §7, GPIO/AFIO §9, ADC §11, TIM1–TIM4 §§14–15, IWDG §17, USART §27.

use super::{Access, Adc, Pins, Uart, checked_rw, command, invalid, ro, rw, unsupported};

#[derive(Clone)]
pub(super) struct Port {
    pub pins: Pins,
    cfg: [u32; 2],
}

impl Default for Port {
    fn default() -> Self {
        Self {
            pins: Pins::default(),
            cfg: [0x4444_4444; 2],
        }
    }
}

impl Port {
    fn mode(&self, bit: u32) -> u32 {
        self.cfg[(bit / 8) as usize] >> ((bit % 8) * 4) & 15
    }
    fn alt_output(&self, bit: u32) -> bool {
        (self.mode(bit) & 15) > 8 && self.mode(bit) & 3 != 0
    }

    fn access(&mut self, o: u32, v: Option<u32>) -> Access {
        match o {
            0 | 4 => {
                let idx = (o / 4) as usize;
                if v.is_some_and(|v| (0..8).any(|n| (v >> (n * 4) & 15) == 12)) {
                    return Err(invalid("зарезервированный режим GPIO STM32"));
                }
                if let Some(value) = v {
                    self.cfg[idx] = value;
                    self.pins.enable = (0..16).fold(0, |all, bit| {
                        all | (u32::from(self.mode(bit) & 3 != 0) << bit)
                    });
                }
                rw(v, &mut self.cfg[idx], u32::MAX)
            }
            8 => {
                let input = (0..16).fold(0, |all, bit| {
                    let mode = self.mode(bit);
                    let pull_up = if mode == 8 {
                        self.pins.out & (1 << bit)
                    } else {
                        0
                    };
                    all | if mode == 0 {
                        0
                    } else {
                        self.pins.read(pull_up) & (1 << bit)
                    }
                });
                ro(v, input)
            }
            0xc => rw(v, &mut self.pins.out, 0xffff),
            0x10 => command(v, |x| {
                self.pins.out = (self.pins.out & !(x >> 16)) | (x & 0xffff)
            }),
            0x14 => command(v, |x| self.pins.out &= !(x & 0xffff)),
            _ => Ok(None),
        }
    }
}

#[derive(Clone)]
struct Timer {
    cr1: u32,
    dier: u32,
    sr: u32,
    ccmr: [u32; 2],
    ccer: u32,
    cnt: u32,
    psc: u32,
    active_psc: u32,
    arr: u32,
    active_arr: u32,
    ccr: [u32; 4],
    active_ccr: [u32; 4],
    phase: u64,
    bdtr: u32,
}

impl Default for Timer {
    fn default() -> Self {
        Self {
            cr1: 0,
            dier: 0,
            sr: 0,
            ccmr: [0; 2],
            ccer: 0,
            cnt: 0,
            psc: 0,
            active_psc: 0,
            arr: 0xffff,
            active_arr: 0xffff,
            ccr: [0; 4],
            active_ccr: [0; 4],
            phase: 0,
            bdtr: 0,
        }
    }
}

impl Timer {
    fn update(&mut self) {
        self.active_arr = self.arr;
        self.active_psc = self.psc;
        self.active_ccr = self.ccr;
        self.sr |= 1;
    }

    fn tick(&mut self, passed: u64) {
        if self.cr1 & 1 == 0 {
            return;
        }
        let div = u64::from(self.active_psc) + 1;
        let tally = passed + self.phase;
        let ticks = (tally / div) as u32;
        self.phase = tally % div;
        if ticks == 0 {
            return;
        }
        for chan in 0..4 {
            let compare = self.active_ccr[chan];
            let distance = if compare > self.cnt {
                compare - self.cnt
            } else {
                (self.active_arr + 1).saturating_sub(self.cnt) + compare
            };
            if compare <= self.active_arr && distance != 0 && ticks >= distance {
                self.sr |= 2 << chan;
            }
        }
        let position = self.cnt + ticks;
        if position > self.active_arr {
            let excess = position - self.active_arr - 1;
            self.update();
            self.cnt = excess % (self.active_arr + 1);
        } else {
            self.cnt = position;
        }
    }

    fn params(&self, channel: usize) -> Option<(u32, u32, bool)> {
        let mode = self.ccmr[channel / 2] >> ((channel % 2) * 8 + 4) & 7;
        if self.cr1 & 1 == 0 || !matches!(mode, 6 | 7) || self.ccer & (1 << (channel * 4)) == 0 {
            return None;
        }
        let compare = self.active_ccr[channel];
        let invert = (mode == 7) ^ (self.ccer & (2 << (channel * 4)) != 0);
        Some((self.active_arr + 1, compare, invert))
    }

    fn access(&mut self, o: u32, v: Option<u32>, advanced: bool) -> Access {
        match o {
            0 => checked_rw(v, &mut self.cr1, 0x81, "TIM CR1 (только счёт вверх)"),
            0xc => checked_rw(v, &mut self.dier, 31, "TIM DIER"),
            0x10 => {
                if let Some(v) = v {
                    self.sr &= v;
                    Ok(Some(0))
                } else {
                    ro(v, self.sr)
                }
            }
            0x14 => match v {
                Some(1) => {
                    self.update();
                    self.cnt = 0;
                    self.phase = 0;
                    Ok(Some(0))
                }
                Some(0) => Ok(Some(0)),
                Some(_) => Err(unsupported("TIM EGR")),
                None => Ok(None),
            },
            0x18 | 0x1c => checked_rw(
                v,
                &mut self.ccmr[((o - 0x18) / 4) as usize],
                0x7878,
                "TIM CCMR (только сравнение и PWM)",
            ),
            0x20 => checked_rw(v, &mut self.ccer, 0x3333, "TIM CCER"),
            0x24 => rw(v, &mut self.cnt, 0xffff),
            0x28 => rw(v, &mut self.psc, 0xffff),
            0x2c => {
                if let Some(v) = v.filter(|_| self.cr1 & 0x80 == 0) {
                    self.active_arr = v & 0xffff;
                }
                rw(v, &mut self.arr, 0xffff)
            }
            0x30 if advanced && (v.is_none() || v == Some(0)) => Ok(Some(0)), // RCR=0.
            0x44 if advanced => checked_rw(v, &mut self.bdtr, 1 << 15, "TIM1 BDTR: только MOE"),
            0x34..=0x40 => {
                let ch = ((o - 0x34) / 4) as usize;
                if let Some(v) = v.filter(|_| self.ccmr[ch / 2] & (8 << ((ch % 2) * 8)) == 0) {
                    self.active_ccr[ch] = v & 0xffff;
                }
                rw(v, &mut self.ccr[ch], 0xffff)
            }
            _ => Ok(None),
        }
    }
}

#[derive(Clone, Default)]
struct Usart {
    brr: u32,
    cr1: u32,
}

#[derive(Clone, Default)]
pub(super) struct Stm {
    pub ports: [Port; 3],
    apb1: u32,
    apb2: u32,
    pub reset_flags: u32,
    lsi: u32,
    adc_psc: u32,
    adc_cr2: u32,
    adc_sample: [u32; 2],
    adc_channel: u32,
    timers: [Timer; 4],
    serial: [Usart; 3],
    watchdog_unlocked: bool,
    watchdog_prescale: u32,
    watchdog_reload: Option<u32>,
    watchdog_deadline: Option<u64>,
}

impl Stm {
    pub fn tick(&mut self, cycles: u64, now: u64) -> bool {
        for (i, t) in self.timers[..3].iter_mut().enumerate() {
            if self.apb1 & (1 << i) != 0 {
                t.tick(cycles);
            }
        }
        if self.apb2 & (1 << 11) != 0 {
            self.timers[3].tick(cycles);
        }
        self.watchdog_deadline.is_some_and(|d| now >= d)
    }

    pub fn interrupts(&self) -> u64 {
        // RM0008, таблица векторов: TIM2/3/4 = IRQ 28/29/30.
        let general = self.timers[..3].iter().enumerate().fold(0, |bits, (i, t)| {
            bits | if self.apb1 & (1 << i) != 0 && t.sr & t.dier != 0 {
                1 << (28 + i)
            } else {
                0
            }
        });
        let tim1 = &self.timers[3];
        let flags = tim1.sr & tim1.dier;
        let advanced = if self.apb2 & 1 << 11 != 0 {
            (u64::from(flags & 1 != 0) << 25) | (u64::from(flags & 0x1e != 0) << 27)
        } else {
            0
        };
        general | advanced
    }

    fn watchdog_feed(&mut self, now: u64) {
        // RM0008: LSI типовое 40 кГц, HSI 8 МГц; оценка без разброса LSI.
        let pr = 4u64 << self.watchdog_prescale.min(6);
        self.watchdog_deadline =
            Some(now + 200 * pr * (u64::from(self.watchdog_reload.unwrap_or(0xfff)) + 1));
    }

    fn timer_pin(&self, pin: u32) -> Option<(&Timer, usize)> {
        let (number, channel) = match pin {
            0..=3 => (0, pin),
            6..=7 => (1, pin - 6),
            8..=11 => (3, pin - 8),
            16..=17 => (1, pin - 14),
            22..=25 => (2, pin - 22),
            _ => return None,
        };
        let port = (pin / 16) as usize;
        let clock = if number == 3 {
            self.apb2 & 1 << 11 != 0 && self.timers[3].bdtr & 1 << 15 != 0
        } else {
            self.apb1 & (1 << number) != 0
        };
        if !clock || !self.ports[port].alt_output(pin % 16) {
            return None;
        }
        Some((&self.timers[number], channel as usize))
    }

    pub fn pwm_period(&self, pin: u32) -> Option<f64> {
        let (timer, _) = self.timer_pin(pin)?;
        // RM0008: счёт вверх, APB без дополнительного деления в этом профиле.
        Some(
            f64::from(timer.active_arr + 1) * f64::from(timer.active_psc + 1)
                / (super::Hardware::frequency(crate::BoardId::Stm32f103c8) as f64),
        )
    }

    pub fn pwm(&self, pin: u32) -> Option<f64> {
        let (timer, channel) = self.timer_pin(pin)?;
        if self.ports[(pin / 16) as usize].mode(pin % 16) & 4 != 0 {
            return None;
        }
        let (period, compare, inverted) = timer.params(channel)?;
        let high = compare.min(period);
        Some(f64::from(if inverted { period - high } else { high }) / f64::from(period))
    }

    pub fn output(&self, pin: u32) -> Option<bool> {
        let p = self.ports.get((pin / 16) as usize)?;
        let bit = pin % 16;
        if p.alt_output(bit) {
            let (t, ch) = self.timer_pin(pin)?;
            let (_, compare, inverted) = t.params(ch)?;
            let high = (t.cnt < compare) ^ inverted;
            if high && p.mode(bit) & 4 != 0 {
                None
            } else {
                Some(high)
            }
        } else if p.mode(bit) & 4 != 0 && p.pins.out & (1 << bit) != 0 {
            None // Открытый сток отпущен.
        } else {
            p.pins.output(bit)
        }
    }

    pub fn access(
        &mut self,
        addr: u32,
        v: Option<u32>,
        now: u64,
        uart: &mut Uart,
        adc: &mut Adc,
    ) -> Access {
        let base = addr & !0x3ff;
        let o = addr & 0x3ff;
        match base {
            0x4002_1000 => match o {
                0 => {
                    // Профиль HSI без переключения SYSCLK и PLL.
                    if v.is_some_and(|x| (x & !2) != 0x81) {
                        return Err(unsupported("RCC CR: только HSI"));
                    }
                    ro(None, if v.is_some() { 0 } else { 0x83 })
                }
                4 => checked_rw(
                    v,
                    &mut self.adc_psc,
                    3 << 14,
                    "RCC CFGR: поддержан только делитель АЦП",
                ),
                0x18 => checked_rw(v, &mut self.apb2, 0x4a01 | 0x1c, "RCC APB2ENR"),
                0x1c => checked_rw(v, &mut self.apb1, 0x60007, "RCC APB1ENR"),
                0x24 => {
                    if let Some(v) = v {
                        if v & 1 << 24 != 0 {
                            self.reset_flags = 0;
                        }
                        self.lsi = v & 1;
                        Ok(Some(0))
                    } else {
                        ro(v, self.reset_flags | self.lsi | self.lsi << 1)
                    }
                }
                _ => Ok(None),
            },
            0x4001_0800 | 0x4001_0c00 | 0x4001_1000 => {
                let idx = ((base - 0x4001_0800) / 0x400) as usize;
                if self.apb2 & (4 << idx) == 0 {
                    return Err(invalid("тактирование GPIO выключено в RCC"));
                }
                self.ports[idx].access(o, v)
            }
            0x4000_0000 | 0x4000_0400 | 0x4000_0800 => {
                let i = ((base / 0x400) & 3) as usize;
                if self.apb1 & (1 << i) == 0 {
                    return Err(invalid("тактирование TIM выключено в RCC"));
                }
                self.timers[i].access(o, v, false)
            }
            0x4001_2c00 => {
                if self.apb2 & 1 << 11 == 0 {
                    return Err(invalid("тактирование TIM1 выключено в RCC"));
                }
                self.timers[3].access(o, v, true)
            }
            0x4001_2400 => {
                if self.apb2 & 1 << 9 == 0 {
                    return Err(invalid("тактирование АЦП выключено в RCC"));
                }
                match o {
                    0 => {
                        if let Some(v) = v {
                            adc.ready &= v & 2 != 0;
                            Ok(Some(0))
                        } else {
                            ro(v, u32::from(adc.ready) << 1)
                        }
                    }
                    4 | 0x2c | 0x30 if v == Some(0) || v.is_none() => Ok(Some(0)), // Одиночная последовательность.
                    8 => {
                        if let Some(v) = v {
                            if v & !0x005e_0001 != 0 {
                                return Err(unsupported("ADC CR2: одиночный запуск от программы"));
                            }
                            let second_adon =
                                v & 1 != 0 && self.adc_cr2 & 1 != 0 && v == self.adc_cr2;
                            self.adc_cr2 = v & !(1 << 22);
                            if v & (1 << 22) != 0 || second_adon {
                                if v & 1 == 0
                                    || adc.deadline.is_some()
                                    || !second_adon && v & 0x001e_0000 != 0x001e_0000
                                {
                                    return Err(invalid(
                                        "ADC1 выключен, занят или выбран неверный триггер",
                                    ));
                                }
                                let ch = self.adc_channel;
                                let (port, bit) = if ch < 8 { (0, ch) } else { (1, ch - 8) };
                                if self.ports[port].mode(bit) != 0 {
                                    return Err(invalid(
                                        "вывод ADC1 должен быть в аналоговом режиме",
                                    ));
                                }
                                // Полутакты ADC из RM0008 SMPR2: t_sample + 12,5.
                                let s = self.adc_sample[1] >> (ch * 3) & 7;
                                let half_ticks =
                                    [3u64, 15, 27, 57, 83, 111, 143, 479][s as usize] + 25;
                                let div = u64::from((self.adc_psc >> 14) + 1) * 2;
                                adc.start(ch as usize, 3.3, 12, now, half_ticks * div / 2)?;
                            }
                            Ok(Some(0))
                        } else {
                            ro(v, self.adc_cr2)
                        }
                    }
                    0xc => rw(v, &mut self.adc_sample[0], 0x00ff_ffff),
                    0x10 => rw(v, &mut self.adc_sample[1], 0x3fff_ffff),
                    0x34 => {
                        if v.is_some_and(|v| v > 9) {
                            Err(unsupported("ADC SQR3 (только внешний одиночный канал)"))
                        } else {
                            rw(v, &mut self.adc_channel, 31)
                        }
                    }
                    0x4c if v.is_none() && adc.ready => {
                        adc.ready = false;
                        ro(v, adc.result)
                    }
                    0x4c if v.is_none() => Err(invalid("результат ADC1 ещё не получен")),
                    _ => Ok(None),
                }
            }
            0x4001_3800 | 0x4000_4400 | 0x4000_4800 => {
                let (i, clock, bank, pin) = match base {
                    0x4001_3800 => (0, self.apb2 & 1 << 14, 0, 9),
                    0x4000_4400 => (1, self.apb1 & 1 << 17, 0, 2),
                    _ => (2, self.apb1 & 1 << 18, 1, 10),
                };
                if clock == 0 {
                    return Err(invalid("тактирование USART выключено в RCC"));
                }
                let port = &mut self.serial[i];
                let tx = &mut uart.ports[i];
                match o {
                    0 => ro(
                        v,
                        (u32::from(tx.queued() == 0) << 7) | (u32::from(!tx.busy()) << 6),
                    ),
                    4 => {
                        if let Some(byte) = v {
                            if port.cr1 & 0x2008 != 0x2008 || !self.ports[bank].alt_output(pin) {
                                return Err(invalid(
                                    "USART требует включённый TX и альтернативную функцию вывода",
                                ));
                            }
                            tx.start(now, 10 * u64::from(port.brr), byte, 1)
                        } else {
                            Ok(None)
                        }
                    }
                    8 => rw(v, &mut port.brr, 0xffff),
                    0xc => checked_rw(v, &mut port.cr1, 0x200c, "USART CR1: только 8N1 TX"),
                    0x10 | 0x14 if v == Some(0) || v.is_none() => Ok(Some(0)),
                    _ => Ok(None),
                }
            }
            0x4000_3000 => match o {
                0 => {
                    if let Some(key) = v {
                        match key {
                            0x5555 => self.watchdog_unlocked = true,
                            0xcccc => self.watchdog_feed(now),
                            0xaaaa => {
                                self.watchdog_unlocked = false;
                                if self.watchdog_deadline.is_some() {
                                    self.watchdog_feed(now);
                                }
                            }
                            _ => return Err(invalid("неверный ключ IWDG")),
                        }
                        Ok(Some(0))
                    } else {
                        Ok(None)
                    }
                }
                4 | 8 if v.is_some() && !self.watchdog_unlocked => {
                    Err(invalid("регистры IWDG закрыты на запись"))
                }
                4 => rw(v, &mut self.watchdog_prescale, 7),
                8 => {
                    if let Some(v) = v {
                        self.watchdog_reload = Some(v & 0xfff);
                        Ok(Some(0))
                    } else {
                        ro(v, self.watchdog_reload.unwrap_or(0xfff))
                    }
                }
                12 => ro(v, 0),
                _ => Ok(None),
            },
            _ => Ok(None),
        }
    }
}
