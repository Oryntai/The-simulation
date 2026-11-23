//! ESP32-C3 TRM: IO MUX/GPIO, Interrupt Matrix, Timer Group, MWDT, UART,
//! LEDC и ADC. Поля и смещения также сверены с заголовками ESP-IDF v5.2.

use super::esp_bus::EspBuses;
use crate::{BoardFault, InputOutcome, PwmOutput};
use std::collections::VecDeque;

use super::{Access, Adc, Pins, Uart, checked_rw, command, invalid, ro, rw, unsupported};

const GPIO_MASK: u32 = (1 << 22) - 1;
const TIMER_MASK: u64 = (1 << 54) - 1;
const WDT_KEY: u32 = 0x50d8_3aa1;

#[derive(Clone)]
struct Group {
    cfg: u32,
    counter: u64,
    latched: u64,
    load: [u32; 2],
    alarm: [u32; 2],
    phase: u64,
    int_en: u32,
    raw: u32,
    wdt_key: u32,
    wdt_setup: u32,
    wdt_div: u32,
    wdt_hold: u32,
    deadline: Option<u64>,
}

impl Default for Group {
    fn default() -> Self {
        Self {
            cfg: 0x6000_2000,
            counter: 0,
            latched: 0,
            load: [0; 2],
            alarm: [0; 2],
            phase: 0,
            int_en: 0,
            raw: 0,
            wdt_key: WDT_KEY,
            wdt_setup: 0x0004_8000,
            wdt_div: 1 << 16,
            wdt_hold: 1_048_575,
            deadline: None,
        }
    }
}

impl Group {
    fn tick(&mut self, cycles: u64, now: u64) -> bool {
        if self.cfg >> 31 != 0 {
            let div = match self.cfg >> 13 & 0xffff {
                0 => 65536,
                1 | 2 => 2,
                n => n,
            };
            let div = u64::from(div) * if self.cfg & (1 << 9) != 0 { 2 } else { 1 };
            self.phase += cycles;
            let ticks = self.phase / div;
            self.phase %= div;
            if ticks != 0 {
                let up = self.cfg & 1 << 30 != 0;
                let target = pair(self.alarm);
                let diff = if up {
                    target.wrapping_sub(self.counter)
                } else {
                    self.counter.wrapping_sub(target)
                } & TIMER_MASK;
                if self.cfg & (1 << 10) != 0 && diff != 0 && diff <= ticks {
                    self.raw |= 1;
                    self.cfg &= !(1 << 10);
                    if self.cfg & 1 << 29 != 0 {
                        self.counter = pair(self.load);
                    } else {
                        self.counter = target;
                    }
                    self.advance(ticks - diff, up);
                } else {
                    self.advance(ticks, up);
                }
            }
        }
        self.deadline.is_some_and(|d| now >= d)
    }

    fn advance(&mut self, ticks: u64, up: bool) {
        self.counter = if up {
            self.counter.wrapping_add(ticks)
        } else {
            self.counter.wrapping_sub(ticks)
        } & TIMER_MASK;
    }

    fn feed(&mut self, now: u64) -> Result<(), crate::BoardFault> {
        if self.wdt_setup >> 31 == 0 {
            self.deadline = None;
            return Ok(());
        }
        let div = self.wdt_div >> 16;
        if div == 0 || self.wdt_hold == 0 {
            return Err(invalid("нулевой делитель или тайм-аут MWDT"));
        }
        let mul = if self.wdt_setup & 1 << 21 != 0 { 2 } else { 1 };
        let delay = u64::from(self.wdt_hold) * u64::from(div) * mul;
        self.deadline = Some(now + delay);
        Ok(())
    }

    fn access(&mut self, o: u32, v: Option<u32>, now: u64) -> Access {
        if (0x48..=0x60).contains(&o) && v.is_some() && self.wdt_key != WDT_KEY {
            return Err(invalid("MWDT заблокирован для записи"));
        }
        match o {
            0 => {
                if v.is_some_and(|v| v & 1 << 12 != 0) {
                    self.phase = 0;
                }
                checked_rw(
                    v.map(|v| v & !(1 << 12)),
                    &mut self.cfg,
                    0xffff_e600,
                    "TIMG T0CONFIG",
                )
            }
            4 => ro(v, self.latched as u32),
            8 => ro(v, (self.latched >> 32) as u32),
            0xc => {
                if v.is_some() {
                    self.latched = self.counter;
                    Ok(Some(0))
                } else {
                    ro(v, 0)
                }
            }
            0x10 | 0x14 => rw(
                v,
                &mut self.alarm[((o - 0x10) / 4) as usize],
                if o == 0x10 { u32::MAX } else { 0x3f_ffff },
            ),
            0x18 | 0x1c => rw(
                v,
                &mut self.load[((o - 0x18) / 4) as usize],
                if o == 0x18 { u32::MAX } else { 0x3f_ffff },
            ),
            0x20 => command(v, |_| self.counter = pair(self.load)),
            0x48 => {
                if let Some(v) = v {
                    if v & !0xe07f_8000 != 0 || v >> 31 != 0 && v >> 29 & 3 != 3 {
                        return Err(unsupported("MWDT: только сброс системы на первой стадии"));
                    }
                    if v >> 31 != 0 && v & 1 << 22 == 0 {
                        return Err(invalid("MWDT требует WDT_CONF_UPDATE_EN"));
                    }
                    self.wdt_setup = v & !(1 << 22);
                    self.feed(now)?;
                    Ok(Some(0))
                } else {
                    ro(v, self.wdt_setup)
                }
            }
            0x4c | 0x50 if self.deadline.is_some() && v.is_some() => Err(unsupported(
                "изменение делителя или порога работающего MWDT",
            )),
            0x4c => rw(v, &mut self.wdt_div, 0xffff_0000),
            0x50 => rw(v, &mut self.wdt_hold, u32::MAX),
            0x60 if v.is_some() => {
                self.feed(now)?;
                Ok(Some(0))
            }
            0x64 => rw(v, &mut self.wdt_key, u32::MAX),
            0x70 => checked_rw(v, &mut self.int_en, 1, "TIMG INT_ENA"),
            0x74 => ro(v, self.raw),
            0x78 => ro(v, self.raw & self.int_en),
            0x7c => command(v, |v| self.raw &= !v),
            _ => Ok(None),
        }
    }
}

fn pair(parts: [u32; 2]) -> u64 {
    u64::from(parts[0]) | (u64::from(parts[1]) << 32)
}

#[derive(Clone, Default)]
struct LedChannel {
    cfg: u32,
    hpoint: u32,
    duty: u32,
    active: u32,
    start: bool,
}
#[derive(Clone)]
struct LedTimer {
    shadow: u32,
    cfg: u32,
    ctr: u32,
    phase: u64,
}

impl Default for LedTimer {
    fn default() -> Self {
        Self {
            shadow: 1 << 23,
            cfg: 1 << 23,
            ctr: 0,
            phase: 0,
        }
    }
}

impl LedTimer {
    fn params(&self, clock: u32) -> Option<(u32, u64)> {
        let div = u64::from(self.cfg >> 4 & 0x3ffff);
        let bits = self.cfg & 15;
        if clock & 3 != 1 || self.cfg & 0x1c0_0000 != 0 || div == 0 || bits == 0 || bits > 14 {
            return None;
        }
        Some((1 << bits, div))
    }

    fn tick(&mut self, cycles: u64, clock: u32) {
        let Some((period, div)) = self.params(clock) else {
            return;
        };
        self.phase += cycles * 256;
        self.ctr = ((u64::from(self.ctr) + self.phase / div) % u64::from(period)) as u32;
        self.phase %= div;
    }
}

// TRM UART: аппаратный FIFO 128 байт; входная очередь хоста на столько же кадров.
pub(super) const UART_FIFO: usize = 128;
const RX_FULL: u32 = 1;
const RX_OVERFLOW: u32 = 1 << 4;

#[derive(Clone)]
struct Serial {
    div: u32,
    config: u32,
    clock: u32,
    wire: VecDeque<(u64, u8)>,
    wire_end_x16: u64,
    rx: VecDeque<u8>,
    threshold: u32,
    overflow: bool,
    enable: u32,
}
impl Default for Serial {
    fn default() -> Self {
        Self {
            div: 0x2b6,
            config: 0x1000_001c,
            clock: 0x0370_1000,
            wire: VecDeque::new(),
            wire_end_x16: 0,
            rx: VecDeque::new(),
            threshold: 96, // UART_CONF1.RXFIFO_FULL_THRHD, reset.
            overflow: false,
            enable: 0,
        }
    }
}

impl Serial {
    fn frame_cycles_x16(&self, transmit: bool) -> Result<u64, BoardFault> {
        if self.config & 0x1000_003f != 0x1000_001c {
            return Err(invalid("UART требует 8N1 и включённую память"));
        }
        let gate = if transmit { 1 << 24 } else { 1 << 25 };
        let gate_mask = gate | (1 << 22);
        let multiplier = match (self.clock >> 20) & 3 {
            1 => 1, // PLL80 в профиле работает на частоте CPU.
            3 => 2, // XTAL 40 МГц.
            _ => return Err(unsupported("источник тактов UART")),
        };
        if self.clock & gate_mask != gate_mask || self.clock & 0xfff != 0 {
            return Err(unsupported("делитель/тактирование UART"));
        }
        let div16 = u64::from((self.div & 0xfff) * 16 + (self.div >> 20 & 15));
        let div = u64::from((self.clock >> 12 & 255) + 1);
        if div16 == 0 {
            return Err(invalid("нулевой делитель UART"));
        }
        // Один стартовый, восемь данных, один стоповый.
        Ok(10 * div16 * div * multiplier)
    }

    fn receive(&mut self, now: u64, bytes: &[u8]) -> Result<InputOutcome, BoardFault> {
        let duration = self.frame_cycles_x16(false)?;
        let accepted = bytes.len().min(UART_FIFO - self.wire.len());
        let mut at = self.wire_end_x16.max(now * 16);
        for &byte in &bytes[..accepted] {
            at += duration;
            self.wire.push_back((at.div_ceil(16), byte));
        }
        self.wire_end_x16 = at;
        Ok(InputOutcome::Uart {
            accepted,
            dropped: bytes.len() - accepted,
        })
    }

    fn tick(&mut self, now: u64) {
        while let Some((_, byte)) = self.wire.pop_front_if(|(at, _)| *at <= now) {
            if self.rx.len() == UART_FIFO {
                self.overflow = true;
            } else {
                self.rx.push_back(byte);
            }
        }
    }

    fn raw(&self) -> u32 {
        (u32::from(self.rx.len() as u32 > self.threshold) * RX_FULL)
            | (u32::from(self.overflow) * RX_OVERFLOW)
    }
}

#[derive(Clone, Default)]
struct Interrupts {
    map: [[u32; 2]; 2],
    serial_bus_map: [u32; 4],
    enabled: u32,
    priority: [u32; 32],
    threshold: u32,
}

#[derive(Clone)]
pub(super) struct Esp {
    pub pins: Pins,
    routes: [u32; 22],
    mux: [u32; 22],
    pin_config: [u32; 22],
    groups: [Group; 2],
    interrupts: Interrupts,
    pub(super) buses: EspBuses,
    channels: [LedChannel; 6],
    led_timers: [LedTimer; 4],
    led_clock: u32,
    serial: [Serial; 2],
    adc_setup: u32,
    adc_irq_cleared: bool,
    adc_sampled: bool,
}

impl Default for Esp {
    fn default() -> Self {
        Self {
            pins: Pins::default(),
            routes: [128; 22],
            mux: [0; 22],
            pin_config: [0; 22],
            groups: std::array::from_fn(|_| Group::default()),
            interrupts: Interrupts::default(),
            buses: EspBuses::default(),
            channels: std::array::from_fn(|_| LedChannel::default()),
            led_timers: std::array::from_fn(|_| LedTimer::default()),
            led_clock: 0,
            serial: std::array::from_fn(|_| Serial::default()),
            adc_setup: 13 << 25,
            adc_irq_cleared: false,
            adc_sampled: false,
        }
    }
}

impl Esp {
    pub fn tick(&mut self, cycles: u64, now: u64) -> bool {
        self.buses.tick(now, false);
        for port in &mut self.serial {
            port.tick(now);
        }
        let a = self.groups[0].tick(cycles, now);
        let b = self.groups[1].tick(cycles, now);
        if self.led_clock != 0 {
            for timer in &mut self.led_timers {
                timer.tick(cycles, self.led_clock);
            }
        }
        a || b
    }

    fn pending(&self) -> u32 {
        let mut mask = 0;
        for (group, map) in self.groups.iter().zip(self.interrupts.map) {
            for (bit, line) in map.into_iter().enumerate() {
                if line != 0 && group.raw & group.int_en & (1 << bit) != 0 {
                    mask |= 1 << line;
                }
            }
        }
        let [i2c, spi] = self.buses.irq();
        let active = [
            self.serial[0].raw() & self.serial[0].enable != 0,
            self.serial[1].raw() & self.serial[1].enable != 0,
            i2c,
            spi,
        ];
        for (active, line) in active.into_iter().zip(self.interrupts.serial_bus_map) {
            if active && line != 0 {
                mask |= 1 << line;
            }
        }
        mask
    }

    pub fn interrupt(&self) -> Result<Option<u32>, crate::BoardFault> {
        let mut bits = self.pending() & self.interrupts.enabled;
        let mut best = None;
        let mut level = 0;
        while bits != 0 {
            let line = bits.trailing_zeros();
            bits &= bits - 1;
            let p = self.interrupts.priority[line as usize];
            if p >= self.interrupts.threshold && p > level {
                best = Some(line);
                level = p;
            }
        }
        Ok(best)
    }

    fn gpio_mux(&self, pin: usize) -> bool {
        self.mux[pin] >> 12 & 7 == 1
    }

    fn pwm_params(&self, pin: usize) -> Option<(&LedTimer, u32, u32, u32)> {
        let route = *self.routes.get(pin)?;
        if !self.gpio_mux(pin)
            || self.pin_config[pin] & 4 != 0
            || route & 0x200 != 0 && self.pins.enable & (1 << pin) == 0
        {
            return None;
        }
        let i = (route & 255).checked_sub(45)?;
        let ch = self.channels.get(i as usize)?;
        let timer = &self.led_timers[(ch.cfg & 3) as usize];
        let (period, _) = timer.params(self.led_clock)?;
        if !ch.start || ch.cfg & 4 == 0 || ch.hpoint >= period {
            return None;
        }
        let high = (ch.active >> 4).min(period);
        Some((timer, period, ch.hpoint, high))
    }

    pub fn pwm(&self, pin: u32) -> Option<PwmOutput> {
        let (timer, period, _, high) = self.pwm_params(pin as usize)?;
        let (_, divider) = timer.params(self.led_clock)?;
        // LEDC: 8 дробных бит делителя, модель работает от APB = CPU.
        let seconds = f64::from(period) * divider as f64
            / (256.0 * super::Hardware::frequency(crate::BoardId::Esp32C3) as f64);
        Some(PwmOutput::new(f64::from(high) / f64::from(period), seconds))
    }

    pub fn uart_receive(
        &mut self,
        port: u32,
        now: u64,
        bytes: &[u8],
    ) -> Result<InputOutcome, BoardFault> {
        self.serial
            .get_mut(port as usize)
            .ok_or_else(|| unsupported("номер UART ESP32-C3"))?
            .receive(now, bytes)
    }

    pub fn output(&self, pin: u32) -> Option<bool> {
        let i = pin as usize;
        let route = *self.routes.get(i)?;
        if let Some((timer, period, start, high)) = self.pwm_params(i) {
            Some((timer.ctr + period - start) % period < high)
        } else if route & 255 == 128 && self.gpio_mux(i) {
            if self.pin_config[i] & 4 != 0 && self.pins.out & (1 << pin) != 0 {
                None
            } else {
                self.pins.output(pin)
            }
        } else {
            None
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
        let o = addr & 0xfff;
        match addr & !0xfff {
            0x6000_4000 => match o {
                4 => rw(v, &mut self.pins.out, GPIO_MASK),
                8 => command(v, |v| self.pins.out |= v & GPIO_MASK),
                0xc => command(v, |v| self.pins.out &= !v),
                0x20 => rw(v, &mut self.pins.enable, GPIO_MASK),
                0x24 => command(v, |v| self.pins.enable |= v & GPIO_MASK),
                0x28 => command(v, |v| self.pins.enable &= !v),
                0x3c => ro(
                    v,
                    (0..22).fold(0, |all, i| {
                        if self.mux[i] & (1 << 9) == 0 {
                            return all;
                        }
                        let pull_up = if self.mux[i] & 0x180 == 0x100 {
                            1 << i
                        } else {
                            0
                        };
                        let high = self
                            .output(i as u32)
                            .unwrap_or(self.pins.input_with_pull(pull_up) & (1 << i) != 0);
                        all | (u32::from(high) << i)
                    }),
                ),
                0x74..=0xc8 => checked_rw(
                    v,
                    &mut self.pin_config[((o - 0x74) / 4) as usize],
                    4,
                    "ESP32-C3 GPIO_PIN: прерывания и sleep",
                ),
                0x554..=0x5a8 => {
                    if v.is_some_and(|v| !matches!(v & 255, 6 | 9 | 45..=50 | 128)) {
                        return Err(unsupported("выбранный выход GPIO Matrix"));
                    }
                    checked_rw(
                        v,
                        &mut self.routes[((o - 0x554) / 4) as usize],
                        0x2ff,
                        "GPIO Matrix инверсия",
                    )
                }
                _ => Ok(None),
            },
            0x6000_9000 if (4..=88).contains(&o) => {
                let pin = (o / 4 - 1) as usize;
                // MCU_SEL=1 включает матрицу GPIO. Только штатный активный режим.
                if v.is_some_and(|v| v >> 12 & 7 != 1) {
                    return Err(unsupported("IO MUX: только GPIO Matrix"));
                }
                checked_rw(v, &mut self.mux[pin], 0x1f80, "IO MUX")
            }
            0x6001_f000 => self.groups[0].access(o, v, now),
            0x6002_0000 => self.groups[1].access(o, v, now),
            0x6001_3000 => self.buses.access(0, o, v, now),
            0x6002_4000 => self.buses.access(2, o, v, now),
            0x600c_2000 => match o {
                // Espressif interrupts.h: UART0/1=21/22, I2C_EXT0=29, SPI2=19.
                0x54 | 0x58 | 0x74 | 0x4c => {
                    let i = match o {
                        0x54 => 0,
                        0x58 => 1,
                        0x74 => 2,
                        _ => 3,
                    };
                    checked_rw(
                        v,
                        &mut self.interrupts.serial_bus_map[i],
                        31,
                        "INTERRUPT MAP",
                    )
                }
                0x80..=0x8c => {
                    let i = ((o - 0x80) / 4) as usize;
                    rw(v, &mut self.interrupts.map[i / 2][i % 2], 31)
                }
                0x104 => rw(v, &mut self.interrupts.enabled, !1),
                0x108 if v == Some(0) || v.is_none() => Ok(Some(0)),
                0x110 => ro(v, self.pending()),
                0x114..=0x190 => rw(
                    v,
                    &mut self.interrupts.priority[((o - 0x114) / 4) as usize],
                    15,
                ),
                0x194 => rw(v, &mut self.interrupts.threshold, 15),
                _ => Ok(None),
            },
            0x6001_9000 => {
                if o < 0x78 {
                    let chan = &mut self.channels[(o / 0x14) as usize];
                    return match o % 0x14 {
                        0 => {
                            if let Some(v) = v {
                                if v & !31 != 0 || v & 8 != 0 {
                                    return Err(unsupported(
                                        "LEDC CONF0: только постоянная скважность",
                                    ));
                                }
                                if v & 16 != 0 {
                                    chan.cfg = v & 7;
                                }
                                Ok(Some(0))
                            } else {
                                ro(v, chan.cfg)
                            }
                        }
                        4 => rw(v, &mut chan.hpoint, 0x3fff),
                        8 => {
                            if v.is_some_and(|v| v & 15 != 0) {
                                return Err(unsupported("дробная скважность LEDC"));
                            }
                            rw(v, &mut chan.duty, 0x7ffff)
                        }
                        12 => {
                            if let Some(v) = v {
                                if v & !0xc000_0000 != 0 {
                                    return Err(unsupported("плавное изменение скважности LEDC"));
                                }
                                if v >> 31 != 0 {
                                    chan.start = true;
                                    chan.active = chan.duty;
                                }
                                Ok(Some(0))
                            } else {
                                ro(v, 1 << 30)
                            }
                        }
                        _ => ro(v, chan.active),
                    };
                }
                match o {
                    0xa0..=0xbc => {
                        let timer = &mut self.led_timers[((o - 0xa0) / 8) as usize];
                        if o & 4 != 0 {
                            return ro(v, timer.ctr);
                        }
                        if let Some(v) = v {
                            if v & !0x2ff_ffff != 0 || v & 15 > 14 {
                                return Err(unsupported("LEDC TIMER CONF"));
                            }
                            timer.shadow = v & 0xff_ffff;
                            if v & (1 << 25) != 0 {
                                timer.cfg = timer.shadow;
                            }
                            if timer.cfg & (1 << 23) != 0 {
                                timer.ctr = 0;
                                timer.phase = 0;
                            }
                            Ok(Some(0))
                        } else {
                            ro(v, timer.shadow)
                        }
                    }
                    0xd0 => {
                        if v.is_some_and(|v| v & 3 > 1) {
                            return Err(unsupported("источник тактов LEDC"));
                        }
                        checked_rw(v, &mut self.led_clock, 0x8000_0003, "LEDC CONF")
                    }
                    _ => Ok(None),
                }
            }
            0x6004_0000 => match o {
                0x20 => {
                    if let Some(v) = v {
                        if v & !0xbf80_0000 != 0 {
                            return Err(unsupported(
                                "ESP32-C3 ADC2 и режимы кроме программного одиночного",
                            ));
                        }
                        if v & 1 << 29 != 0 && self.adc_setup & 1 << 29 == 0 {
                            let channel = (v >> 25 & 15) as usize;
                            if v & 1 << 31 == 0 || channel >= 5 || adc.deadline.is_some() {
                                return Err(invalid(
                                    "ADC1 выключен, занят или выбран недопустимый канал",
                                ));
                            }
                            let attenuation = (v >> 23 & 3) as usize;
                            let volts = adc.volts[channel]
                                .ok_or_else(|| invalid("не задан вход ADC1 ESP32-C3"))?;
                            // Datasheet таблица 5-6: границы откалиброванного рабочего диапазона.
                            if !(0.0..=[0.75, 1.05, 1.3, 2.5][attenuation]).contains(&volts) {
                                return Err(invalid(
                                    "ADC1 ESP32-C3 вне диапазона выбранного ослабления",
                                ));
                            }
                            // TRM §34.2.4: номинальные Vref и коэффициенты ослабления.
                            // Оценка raw-кода, калибровка eFuse и нелинейность отсутствуют.
                            let gain = [1.0, 0.75, 0.5, 0.25][attenuation];
                            adc.result = (volts * gain / 1.1 * 4095.0).floor() as u32;
                            // Таблица 5-5 даташита приводит до 100 ksps для DIG ADC.
                            // Оценка 10 мкс на отсчёт, не гарантия латентности one-shot.
                            adc.ready = false;
                            adc.deadline = Some(now + 800);
                            self.adc_irq_cleared = false;
                            self.adc_sampled = true;
                        }
                        self.adc_setup = v;
                        Ok(Some(0))
                    } else {
                        ro(v, self.adc_setup)
                    }
                }
                0x2c if self.adc_sampled && adc.ready => ro(v, adc.result),
                0x2c => Err(invalid("результат SAR ADC1 ещё не получен")),
                0x44 => ro(
                    v,
                    u32::from(self.adc_sampled && adc.ready && !self.adc_irq_cleared) << 31,
                ),
                0x4c => command(v, |v| {
                    if v >> 31 != 0 && adc.ready {
                        self.adc_irq_cleared = true;
                    }
                }),
                _ => Ok(None),
            },
            0x6000_0000 | 0x6001_0000 => {
                let i = usize::from(addr >= 0x6001_0000);
                let serial = &mut self.serial[i];
                let tx = &mut uart.ports[i];
                if v.is_some() && !serial.wire.is_empty() && matches!(o, 0x14 | 0x20 | 0x78) {
                    return Err(unsupported("изменение UART во время приёма кадра"));
                }
                match o {
                    0 if v.is_some() => {
                        let signal = if i == 0 { 6 } else { 9 };
                        if serial.config & 0x1000_003f != 0x1000_001c
                            || !self.routes.iter().enumerate().any(|(pin, r)| {
                                r & 255 == signal
                                    && self.mux[pin] >> 12 & 7 == 1
                                    && (r & 0x200 == 0 || self.pins.enable & (1 << pin) != 0)
                            })
                        {
                            return Err(invalid("UART требует 8N1 и маршрут TX через GPIO Matrix"));
                        }
                        tx.start(
                            now,
                            serial.frame_cycles_x16(true)?.div_ceil(16),
                            v.unwrap(),
                            UART_FIFO,
                        )
                    }
                    0 => serial
                        .rx
                        .pop_front()
                        .map(|b| Some(u32::from(b)))
                        .ok_or_else(|| invalid("UART: RX FIFO пуст")),
                    4 => ro(v, serial.raw()),
                    8 => ro(v, serial.raw() & serial.enable),
                    0xc => checked_rw(v, &mut serial.enable, RX_FULL | RX_OVERFLOW, "UART INT_ENA"),
                    0x10 => command(v, |bits| {
                        if bits & RX_OVERFLOW != 0 {
                            serial.overflow = false;
                        }
                    }),
                    0x14 => rw(v, &mut serial.div, 0x00f0_0fff),
                    0x1c => ro(v, ((tx.queued() as u32) << 16) | serial.rx.len() as u32),
                    0x20 => {
                        let result = checked_rw(
                            v,
                            &mut serial.config,
                            0x1202_003f,
                            "UART CONF0: только 8N1",
                        )?;
                        if v.is_some_and(|value| value & (1 << 17) != 0) {
                            serial.rx.clear();
                            serial.config &= !(1 << 17);
                        }
                        Ok(result)
                    }
                    0x24 => {
                        if v.is_some_and(|n| n >= UART_FIFO as u32) {
                            return Err(unsupported("UART CONF1: только порог RX FIFO"));
                        }
                        checked_rw(v, &mut serial.threshold, 0x1ff, "UART CONF1")
                    }
                    0x6c => ro(v, u32::from(tx.busy()) << 4),
                    0x78 => checked_rw(v, &mut serial.clock, 0x037f_f000, "UART CLK_CONF"),
                    _ => Ok(None),
                }
            }
            _ => Ok(None),
        }
    }
}
