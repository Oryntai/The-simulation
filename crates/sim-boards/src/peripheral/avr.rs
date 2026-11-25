//! ATmega328P 7810D: §§10–11 сброс и прерывания, §14 порты, §§15–17
//! Timer0/1/2, §20 USART0, §24 АЦП. Адреса в пространстве данных.

use super::{Access, Adc, Pins, Uart, checked_rw, invalid, ro, rw, unsupported};

pub(super) fn pin_location(pin: u32) -> Option<(usize, u32)> {
    match pin {
        0..=7 => Some((2, pin)),
        8..=13 => Some((0, pin - 8)),
        14..=19 => Some((1, pin - 14)),
        _ => None,
    }
}

#[derive(Clone, Copy)]
enum Mode {
    Normal,
    Ctc(u32),
    Fast(u32),
    Phase(u32),
}

#[derive(Clone, Default)]
struct Timer {
    a: u32,
    b: u32,
    cnt: u32,
    compare: [u32; 2],
    active: [u32; 2],
    capture: u32,
    mask: u32,
    flags: u32,
    phase: u64,
    down: bool,
    temporary_high: u8,
}

impl Timer {
    fn mode(a: u32, b: u32, id: usize, capture: u32, compare_a: u32) -> Option<Mode> {
        let low = a & 3;
        if id == 2 {
            // 7810D таблица 16-4, WGM13:0 Timer1.
            match low | (b >> 1 & 12) {
                0 => Some(Mode::Normal),
                1..=3 => Some(Mode::Phase((1 << (7 + low)) - 1)),
                4 => Some(Mode::Ctc(compare_a)),
                5..=7 => Some(Mode::Fast((1 << (7 + low)) - 1)),
                12 => Some(Mode::Ctc(capture)),
                14 => Some(Mode::Fast(capture)),
                _ => None,
            }
        } else {
            // Timer0/2: поддержаны режимы с фиксированным PWM TOP и CTC.
            match low | (b >> 1 & 4) {
                0 => Some(Mode::Normal),
                1 => Some(Mode::Phase(255)),
                2 => Some(Mode::Ctc(compare_a)),
                3 => Some(Mode::Fast(255)),
                _ => None,
            }
        }
    }

    fn selected_mode(&self, id: usize) -> Option<Mode> {
        Self::mode(self.a, self.b, id, self.capture, self.active[0])
    }

    fn divisor(&self, id: usize) -> Option<u64> {
        let list = if id == 1 {
            [0, 1, 8, 32, 64, 128, 256, 1024]
        } else {
            [0, 1, 8, 64, 256, 1024, 0, 0]
        };
        let div = list[(self.b & 7) as usize];
        (div != 0).then_some(div)
    }

    fn tick(&mut self, cycles: u64, id: usize) {
        let Some(div) = self.divisor(id) else {
            return;
        };
        let mask = if id == 2 { 0xffff } else { 255 };
        self.phase += cycles;
        let ticks = self.phase / div;
        self.phase %= div;
        for _ in 0..ticks {
            let Some(mode) = self.selected_mode(id) else {
                return;
            };
            let max = match mode {
                Mode::Normal => mask,
                Mode::Ctc(top) | Mode::Fast(top) | Mode::Phase(top) => top,
            };
            // Сравнение проверяется для состояния счётчика до перехода на следующий тик.
            for (i, &compare) in self.active.iter().enumerate() {
                if self.cnt == compare {
                    self.flags |= 2 << i;
                }
            }
            if let Mode::Phase(top) = mode {
                if self.cnt == 0 {
                    self.flags |= 1;
                    self.down = false;
                } else if self.cnt == top {
                    self.active = self.compare;
                    self.down = true;
                }
                self.cnt = if self.down {
                    self.cnt.wrapping_sub(1) & mask
                } else {
                    self.cnt.wrapping_add(1) & mask
                };
                continue;
            }
            if self.cnt == mask || matches!(mode, Mode::Fast(top) if self.cnt == top) {
                self.flags |= 1;
            }
            if self.cnt == max || self.cnt == mask {
                self.cnt = 0;
                if matches!(mode, Mode::Fast(_)) {
                    self.active = self.compare;
                }
            } else {
                self.cnt += 1;
            }
        }
    }

    fn pwm_params(&self, ch: usize, id: usize) -> Option<(Mode, bool)> {
        let com = self.a >> (6 - 2 * ch) & 3;
        if self.divisor(id).is_none() || com < 2 {
            return None;
        }
        let mode = self.selected_mode(id)?;
        if !matches!(mode, Mode::Fast(_) | Mode::Phase(_)) {
            return None;
        }
        Some((mode, com == 3))
    }

    fn pwm_duty(&self, ch: usize, id: usize) -> Option<f64> {
        let (mode, inverted) = self.pwm_params(ch, id)?;
        let (high, total) = match mode {
            // Fast PWM: OCR=BOTTOM даёт одноклоковый импульс.
            Mode::Fast(max) => ((self.active[ch] + 1).min(max + 1), max + 1),
            Mode::Phase(max) if max != 0 => (self.active[ch].min(max), max),
            _ => return None,
        };
        Some(f64::from(if inverted { total - high } else { high }) / f64::from(total))
    }

    fn pwm_level(&self, ch: usize, id: usize) -> Option<bool> {
        let (mode, inverted) = self.pwm_params(ch, id)?;
        let compare = self.active[ch];
        let level = match mode {
            Mode::Fast(max) => compare >= max || self.cnt <= compare,
            Mode::Phase(max) => {
                compare >= max
                    || compare != 0 && (self.cnt < compare || self.down && self.cnt == compare)
            }
            _ => return None,
        };
        Some(level ^ inverted)
    }

    fn configuration(&mut self, is_b: bool, v: Option<u32>, id: usize) -> Access {
        let Some(v) = v else {
            return ro(v, if is_b { self.b } else { self.a });
        };
        let (a, b) = if is_b { (self.a, v) } else { (v, self.b) };
        let mode = Self::mode(a, b, id, self.capture, self.compare[0]);
        let running = b & 7 != 0;
        if a & !0xf3 != 0
            || b & !(if id == 2 { 0x1f } else { 0x0f }) != 0
            || id != 1 && b & 7 >= 6
            || running
                && (mode.is_none()
                    || matches!(mode, Some(Mode::Fast(0) | Mode::Phase(0)))
                    || [a >> 6 & 3, a >> 4 & 3].into_iter().any(|com| {
                        if matches!(mode, Some(Mode::Fast(_) | Mode::Phase(_))) {
                            com == 1
                        } else {
                            com != 0
                        }
                    }))
        {
            return Err(unsupported(
                "режимы AVR Timer0/1/2, не входящие в Normal, CTC и поддержанные PWM",
            ));
        }
        if b != self.b {
            self.phase = 0;
        }
        if a != self.a || b != self.b && !running {
            self.active = self.compare;
            self.down = false;
        }
        self.a = a;
        self.b = b;
        Ok(Some(0))
    }

    fn word(&mut self, o: u32, id: usize, v: Option<u32>) -> Access {
        let (index, high) = if id == 2 {
            ((o - 4) / 2, o & 1 != 0)
        } else {
            (o - 2 + u32::from(o > 2), false)
        };
        let register = match index {
            0 => self.cnt,
            1 if id == 2 => self.capture,
            2 | 3 => self.compare[(index - 2) as usize],
            _ => return Ok(None),
        };
        if let Some(v) = v {
            let value = if high {
                self.temporary_high = v as u8;
                return Ok(Some(0));
            } else if id == 2 {
                (u32::from(self.temporary_high) << 8) | v
            } else {
                v
            };
            match index {
                0 => self.cnt = value,
                1 => {
                    if self.divisor(id).is_some()
                        && matches!(self.selected_mode(id), Some(Mode::Fast(_)))
                    {
                        return Err(unsupported("изменение ICR1 в работающем Fast PWM"));
                    }
                    self.capture = value;
                }
                _ => {
                    self.compare[(index - 2) as usize] = value;
                    if self.divisor(id).is_none()
                        || !matches!(self.selected_mode(id), Some(Mode::Phase(_) | Mode::Fast(_)))
                    {
                        self.active = self.compare;
                    }
                }
            }
            Ok(Some(0))
        } else if high {
            // 7810D: у OCR1A/B чтение не использует общий TEMP.
            ro(
                v,
                if index <= 1 {
                    u32::from(self.temporary_high)
                } else {
                    register >> 8
                },
            )
        } else {
            if id == 2 && index <= 1 {
                self.temporary_high = (register >> 8) as u8;
            }
            ro(v, register & 255)
        }
    }

    fn access(&mut self, o: u32, v: Option<u32>, id: usize) -> Access {
        match o {
            0 | 1 => self.configuration(o == 1, v, id),
            4..=11 if id == 2 => self.word(o, id, v),
            2 | 3 if id != 2 => self.word(o, id, v),
            4 if id != 2 => self.word(o, id, v),
            _ => Ok(None),
        }
    }
}

#[derive(Clone, Default)]
pub(super) struct Avr {
    pub ports: [Pins; 3],
    pub mcusr: u32,
    timers: [Timer; 3],
    wdt: u32,
    unlock_until: Option<u64>,
    watchdog_deadline: Option<u64>,
    watchdog_pending: bool,
    admux: u32,
    adcsra: u32,
    adc_first: bool,
    adc_data: Option<u32>,
    adc_irq_flag: bool,
    adc_locked: bool,
    baud: u32,
    uart_b: u32,
    uart_c: Option<u32>,
    uart_a: u32,
    was_tx: bool,
}

impl Avr {
    pub fn adc_complete(&mut self, result: u32) {
        self.adc_irq_flag = true;
        // У ATmega328P чтение ADCL блокирует обновление пары до чтения ADCH.
        if !self.adc_locked {
            self.adc_data = Some(if self.admux & 0x20 != 0 {
                result << 6
            } else {
                result
            });
        }
    }

    pub fn watchdog_after_reset(&mut self, now: u64) {
        self.wdt = 8;
        self.feed(now);
    }

    pub fn feed(&mut self, now: u64) {
        if self.wdt & 0x48 != 0 {
            let select = self.wdt & 7 | (self.wdt >> 2 & 8);
            // 7810D таблица 10-5: 2K .. 1024K тактов WDT, 128 кГц тип., CPU 16 МГц.
            self.watchdog_deadline = Some(now + ((2048u64 * 125) << select));
        } else {
            self.watchdog_deadline = None;
        }
    }

    pub fn tick(&mut self, cycles: u64, now: u64) -> bool {
        for (id, timer) in self.timers.iter_mut().enumerate() {
            timer.tick(cycles, id);
        }
        if self.watchdog_deadline.is_none_or(|d| now < d) {
            return false;
        }
        if self.wdt & 0x40 != 0 {
            self.watchdog_pending = true;
            if self.wdt & 8 != 0 {
                self.wdt &= !0x40;
            }
            self.feed(now);
            return false;
        }
        self.wdt & 8 != 0
    }

    pub fn interrupt(&mut self) -> Option<u32> {
        if std::mem::take(&mut self.watchdog_pending) {
            return Some(0x18);
        }
        for (i, base) in [(1, 0x1c), (2, 0x2c), (0, 0x38)] {
            let timer = &mut self.timers[i];
            for (n, flag) in [2, 4, 1].into_iter().enumerate() {
                if timer.flags & timer.mask & flag != 0 {
                    timer.flags &= !flag;
                    return Some(base + n as u32 * 4);
                }
            }
        }
        None
    }

    fn pwm_pin(&self, pin: u32) -> Option<(&Timer, usize, usize)> {
        let (port, bit) = pin_location(pin)?;
        if self.ports[port].enable & (1 << bit) == 0 {
            return None;
        }
        let (i, ch) = match pin {
            6 => (0, 0),
            5 => (0, 1),
            11 => (1, 0),
            3 => (1, 1),
            9 => (2, 0),
            10 => (2, 1),
            _ => return None,
        };
        Some((&self.timers[i], ch, i))
    }

    pub fn output(&self, pin: u32) -> Option<bool> {
        let (port, bit) = pin_location(pin)?;
        if let Some((timer, ch, id)) = self.pwm_pin(pin)
            && let Some(level) = timer.pwm_level(ch, id)
        {
            return Some(level);
        }
        self.ports[port].output(bit)
    }

    pub(super) fn gpio_mode(&self, pin: u32) -> Option<crate::GpioMode> {
        use crate::GpioMode::*;
        let (port, bit) = pin_location(pin)?;
        // 7810D, GPIO: PORTxn при DDRxn = 0 включает внутреннюю подтяжку.
        let pins = self.ports[port];
        if pins.enable & (1 << bit) != 0 {
            return self.output(pin).map(Output);
        }
        Some(if pins.out & (1 << bit) != 0 {
            PullUp
        } else {
            Input
        })
    }

    pub fn pwm_period(&self, pin: u32) -> Option<f64> {
        let (timer, ch, id) = self.pwm_pin(pin)?;
        let (mode, _) = timer.pwm_params(ch, id)?;
        // 7810D: 1+TOP для Fast PWM; 2·TOP для счёта вверх и вниз.
        let ticks = match mode {
            Mode::Fast(top) => top + 1,
            Mode::Phase(top) if top > 0 => 2 * top,
            _ => return None,
        };
        Some(
            f64::from(ticks) * timer.divisor(id)? as f64
                / (super::Hardware::frequency(crate::BoardId::Atmega328p) as f64),
        )
    }

    pub fn pwm(&self, pin: u32) -> Option<f64> {
        let (timer, ch, id) = self.pwm_pin(pin)?;
        timer.pwm_duty(ch, id)
    }

    pub fn access(
        &mut self,
        address: u32,
        v: Option<u32>,
        now: u64,
        uart: &mut Uart,
        adc: &mut Adc,
    ) -> Access {
        match address {
            0x23..=0x2b => {
                let port = &mut self.ports[((address - 0x23) / 3) as usize];
                let mask = if (0x26..=0x28).contains(&address) {
                    0x7f
                } else {
                    0xff
                };
                match (address - 0x23) % 3 {
                    0 => {
                        if let Some(v) = v {
                            port.out ^= v & mask;
                            Ok(Some(0))
                        } else {
                            ro(v, port.read(port.out) & mask)
                        }
                    }
                    1 => rw(v, &mut port.enable, mask),
                    _ => rw(v, &mut port.out, mask),
                }
            }
            0x35..=0x37 => {
                let t = &mut self.timers[[0, 2, 1][(address - 0x35) as usize]];
                if let Some(v) = v {
                    t.flags &= !v;
                    Ok(Some(0))
                } else {
                    ro(v, t.flags)
                }
            }
            0x44..=0x48 => self.timers[0].access(address - 0x44, v, 0),
            0xb0..=0xb4 => self.timers[1].access(address - 0xb0, v, 1),
            0x80..=0x8b => self.timers[2].access(address - 0x80, v, 2),
            0x6e..=0x70 => checked_rw(
                v,
                &mut self.timers[[0, 2, 1][(address - 0x6e) as usize]].mask,
                7,
                "прерывания AVR Timer0/1/2",
            ),
            0x54 => {
                if let Some(v) = v {
                    self.mcusr &= v;
                    Ok(Some(0))
                } else {
                    ro(v, self.mcusr)
                }
            }
            0x60 => {
                if let Some(v) = v {
                    if v & 0x80 != 0 {
                        self.watchdog_pending = false;
                    }
                    if v & 0x18 == 0x18 {
                        // WDCE и WDE открывают четыре такта для изменения.
                        self.unlock_until = Some(now + 4);
                        self.wdt |= 8;
                        if self.watchdog_deadline.is_none() {
                            self.feed(now);
                        }
                    } else if self.unlock_until.is_some_and(|t| now <= t) {
                        self.unlock_until = None;
                        if (v & 7 | (v >> 2 & 8)) > 9 {
                            return Err(invalid("зарезервированный предделитель WDT"));
                        }
                        self.wdt = (v & 0x6f) | if self.mcusr & 8 != 0 { 8 } else { 0 };
                        self.feed(now);
                    } else if (self.wdt ^ v) & 0x2f != 0 {
                        return Err(invalid("изменение WDTCSR вне окна WDCE"));
                    } else {
                        self.wdt = (self.wdt & !0x40) | (v & 0x40);
                        self.feed(now);
                    }
                    Ok(Some(0))
                } else {
                    ro(
                        v,
                        self.wdt
                            | (u32::from(self.watchdog_pending) << 7)
                            | (u32::from(self.unlock_until.is_some_and(|t| now <= t)) << 4),
                    )
                }
            }
            0x78 | 0x79 if v.is_none() => {
                let data = self
                    .adc_data
                    .ok_or_else(|| invalid("результат АЦП ещё не получен"))?;
                self.adc_locked = address == 0x78;
                ro(
                    v,
                    if self.adc_locked {
                        data & 255
                    } else {
                        data >> 8
                    },
                )
            }
            0x7a => {
                if let Some(v) = v {
                    if v & 0x28 != 0 {
                        return Err(unsupported("AVR ADC: автостарт и прерывание"));
                    }
                    if v & 0x80 == 0 {
                        adc.deadline = None;
                        self.adc_first = false;
                    }
                    if v & 0x80 != 0 && self.adcsra & 0x80 == 0 {
                        self.adc_first = true;
                    }
                    self.adcsra = v & 0x87;
                    if v & 0x10 != 0 {
                        self.adc_irq_flag = false;
                    }
                    if v & 0x40 != 0 && adc.deadline.is_none() {
                        if v & 0x80 == 0 {
                            return Err(invalid("АЦП выключен"));
                        }
                        let reference = match self.admux >> 6 {
                            1 => 5.0,
                            3 => 1.1,
                            _ => return Err(unsupported("внешняя опора AREF")),
                        };
                        let ch = self.admux & 15;
                        if ch > 7 {
                            return Err(unsupported("внутренний канал AVR ADC"));
                        }
                        let div = 1u64 << (v & 7).max(1);
                        // 7810D §23.4: 50–200 кГц для полной разрядности.
                        if 16_000_000 / div > 200_000 {
                            return Err(unsupported(
                                "AVR ADC: частота выше предела для полной разрядности",
                            ));
                        }
                        adc.start(
                            ch as usize,
                            reference,
                            10,
                            now,
                            (if self.adc_first { 25 } else { 13 }) * div,
                        )?;
                        self.adc_first = false;
                    }
                    Ok(Some(0))
                } else {
                    ro(
                        v,
                        self.adcsra
                            | (u32::from(adc.deadline.is_some()) << 6)
                            | (u32::from(self.adc_irq_flag) << 4),
                    )
                }
            }
            0x7b if v == Some(0) || v.is_none() => Ok(Some(0)),
            0x7c if v.is_some() && adc.deadline.is_some() => {
                Err(unsupported("запись ADMUX во время преобразования"))
            }
            0x7c => checked_rw(v, &mut self.admux, 0xef, "AVR ADMUX"),
            0xc0 => {
                if let Some(v) = v {
                    if v & !0x42 != 0 {
                        return Err(unsupported("AVR UCSR0A"));
                    }
                    self.uart_a = v & 2;
                    if v & 0x40 != 0 {
                        self.was_tx = false;
                    }
                    Ok(Some(0))
                } else {
                    ro(
                        v,
                        self.uart_a
                            | (u32::from(uart.ports[0].queued() == 0) << 5)
                            | (u32::from(self.was_tx && !uart.ports[0].busy()) << 6),
                    )
                }
            }
            0xc1 => checked_rw(v, &mut self.uart_b, 0x18, "AVR UCSR0B (режим 8N1)"),
            0xc2 => {
                if let Some(v) = v {
                    if v != 6 {
                        return Err(unsupported("AVR UCSR0C: только 8N1"));
                    }
                    self.uart_c = Some(v);
                    Ok(Some(0))
                } else {
                    ro(v, self.uart_c.unwrap_or(6))
                }
            }
            0xc4 | 0xc5 => {
                if let Some(v) = v {
                    if address == 0xc4 {
                        self.baud = (self.baud & 0xf00) | v;
                    } else {
                        self.baud = (self.baud & 0xff) | ((v & 15) << 8);
                    }
                    Ok(Some(0))
                } else {
                    ro(
                        v,
                        if address == 0xc4 {
                            self.baud & 255
                        } else {
                            self.baud >> 8
                        },
                    )
                }
            }
            0xc6 if v.is_some() => {
                if self.uart_b & 8 == 0 {
                    return Err(invalid("AVR USART0 TX выключен"));
                }
                self.was_tx = true;
                let div = if self.uart_a & 2 != 0 { 8 } else { 16 };
                uart.ports[0].start(now, u64::from((self.baud + 1) * div * 10), v.unwrap(), 1)
            }
            _ => Ok(None),
        }
    }
}
