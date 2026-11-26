mod avr;
mod esp;
pub mod esp_adc;
mod esp_assist;
mod esp_bus;
pub mod esp_dma;
pub mod esp_flash;
pub(crate) mod esp_pms;
mod esp_regi2c;
pub(crate) mod esp_rtc;
mod esp_rtc_cal;
mod esp_sar_digital;
mod esp_sha;
pub(crate) mod esp_sleep;
mod esp_sleep_regs;
pub(crate) mod esp_system;
mod esp_timg;
mod esp_uart;
mod pico;
mod stm;

use crate::esp_memory::EspMemory;
use crate::{
    BoardFault, BoardId, BoardInput, BoardOutput, GpioMode, InputOutcome, OutputEvent, PwmOutput,
    ResetCause,
};

type Access = Result<Option<u32>, BoardFault>;

pub(crate) fn supports_reset_value(address: u32, field: &str, value: u32) -> bool {
    esp_regi2c::supports_reset_value(address, field, value)
        || esp_flash::supports_profile_byte(address, field, value)
        || esp_sleep_regs::supports_reset_value(address, field, value)
}

pub(crate) fn ro(v: Option<u32>, actual: u32) -> Access {
    Ok(v.is_none().then_some(actual))
}

fn command(v: Option<u32>, act: impl FnOnce(u32)) -> Access {
    Ok(v.map(|v| {
        act(v);
        0
    }))
}

fn rw(v: Option<u32>, dst: &mut u32, mask: u32) -> Access {
    Ok(Some(if let Some(v) = v {
        *dst = v & mask;
        0
    } else {
        *dst
    }))
}

fn checked_rw(v: Option<u32>, dst: &mut u32, mask: u32, block: &str) -> Access {
    if v.is_some_and(|v| v & !mask != 0) {
        return Err(unsupported(block));
    }
    rw(v, dst, mask)
}

fn unsupported(block: &str) -> BoardFault {
    BoardFault::Unsupported(format!("режим блока {block}"))
}

fn invalid(message: &str) -> BoardFault {
    BoardFault::Misconfigured(message.into())
}

#[derive(Clone, Copy, Default)]
struct Pins {
    input: u32,
    driven: u32,
    out: u32,
    enable: u32,
}

impl Pins {
    fn input_with_pull(self, pull_up: u32) -> u32 {
        (self.input & self.driven) | (pull_up & !self.driven)
    }
    fn read(self, pull_up: u32) -> u32 {
        (self.input_with_pull(pull_up) & !self.enable) | (self.enable & self.out)
    }
    fn keep_input(&mut self, old: Self) {
        self.input = old.input;
        self.driven = old.driven;
    }
    fn output(self, bit: u32) -> Option<bool> {
        (self.enable & (1 << bit) != 0).then_some(self.out & (1 << bit) != 0)
    }
}

#[derive(Clone, Default)]
struct Adc {
    volts: [Option<f64>; 18],
    deadline: Option<u64>,
    result: u32,
    ready: bool,
}

impl Adc {
    fn start(
        &mut self,
        channel: usize,
        full_scale: f64,
        bits: u32,
        cycles: u64,
        duration: u64,
    ) -> Result<(), BoardFault> {
        let volts = self
            .volts
            .get(channel)
            .copied()
            .flatten()
            .ok_or_else(|| invalid("для выбранного канала АЦП не задано входное напряжение"))?;
        if !volts.is_finite() || !(0.0..=full_scale).contains(&volts) {
            return Err(invalid(
                "входное напряжение вне смоделированного диапазона АЦП",
            ));
        }
        // Идеальный прямой двоичный АЦП: переходы через Vref / 2^N, верхний код насыщается.
        let bins = 1 << bits;
        self.result = ((volts / full_scale * f64::from(bins)) as u32).min(bins - 1);
        self.ready = false;
        self.deadline = Some(cycles.saturating_add(duration));
        Ok(())
    }

    fn tick(&mut self, cycles: u64) -> bool {
        let completed = self.deadline.is_some_and(|t| cycles >= t);
        if completed {
            self.ready = true;
            self.deadline = None;
        }
        completed
    }
}

#[derive(Clone, Default)]
struct TxPort {
    tx: Option<(u64, u8)>,
    fifo: std::collections::VecDeque<(u64, u8)>,
}

impl TxPort {
    fn busy(&self) -> bool {
        self.tx.is_some()
    }
    fn queued(&self) -> usize {
        self.fifo.len()
    }

    fn start(&mut self, now: u64, duration: u64, byte: u32, fifo_capacity: usize) -> Access {
        if duration == 0 {
            return Err(invalid("отсутствует тактирование UART"));
        }
        if self.tx.is_none() {
            self.tx = Some((now + duration, byte as u8));
        } else if self.queued() < fifo_capacity {
            self.fifo.push_back((duration, byte as u8));
        } else {
            return Err(invalid("буфер передатчика UART заполнен"));
        }
        Ok(Some(0))
    }

    fn tick(&mut self, now: u64) -> Option<(u64, u8)> {
        let (at, byte) = self.tx.filter(|&(at, _)| now >= at)?;
        self.tx = self
            .fifo
            .pop_front()
            .map(|(duration, next)| (at + duration, next));
        Some((at, byte))
    }
}

#[derive(Clone, Default)]
struct Uart {
    done: Vec<OutputEvent>,
    ports: [TxPort; 3], // STM32F103 USART1…3.
}

impl Uart {
    fn tick(&mut self, now: u64) {
        if !self
            .ports
            .iter()
            .any(|p| p.tx.is_some_and(|(at, _)| now >= at))
        {
            return;
        }
        let mut completed = Vec::new();
        for (number, port) in self.ports.iter_mut().enumerate() {
            while let Some((cycle, byte)) = port.tick(now) {
                completed.push(OutputEvent {
                    cycle,
                    output: BoardOutput::Uart {
                        port: number as u32,
                        byte,
                    },
                });
            }
        }
        completed.sort_by_key(|event| event.cycle);
        self.done.extend(completed);
    }
}

#[derive(Clone)]
enum Chip {
    Pico(Box<pico::Pico>),
    Stm(Box<stm::Stm>),
    Avr(Box<avr::Avr>),
    Esp(Box<esp::Esp>),
}

#[derive(Clone)]
pub(crate) struct Hardware {
    id: BoardId,
    chip: Chip,
    rtc: Option<Box<esp_rtc::EspRtc>>,
    pub(crate) esp_boot: Option<Box<esp::BootPeripherals>>,
    esp_sleep: Option<Box<esp_sleep::EspSleep>>,
    pub cycles: u64,
    pub vector_base: u32,
    pub arm_active: u16,
    pub arm_active_mask: u64,
    nvic: Nvic,
    uart: Uart,
    adc: Adc,
    pub reset_requested: bool,
    pub watchdog_resets: u32,
    pub reset_events: Vec<OutputEvent>,
}

impl Hardware {
    pub fn new(id: BoardId) -> Self {
        let (chip, vector_base) = match id {
            BoardId::Virtual => unreachable!("виртуальный профиль имеет собственную периферию"),
            BoardId::Rp2040 => (Chip::Pico(Box::default()), 0x1000_0000),
            BoardId::Stm32f103c8 => (Chip::Stm(Box::default()), 0),
            BoardId::Atmega328p => (Chip::Avr(Box::default()), 0),
            BoardId::Esp32C3 => (Chip::Esp(Box::default()), 0),
        };
        Self {
            id,
            chip,
            rtc: (id == BoardId::Esp32C3).then(Box::default),
            esp_boot: None,
            esp_sleep: None,
            cycles: 0,
            vector_base,
            arm_active: 0,
            arm_active_mask: 0,
            nvic: Nvic::default(),
            uart: Uart::default(),
            adc: Adc::default(),
            reset_requested: false,
            watchdog_resets: 0,
            reset_events: Vec::new(),
        }
    }

    pub fn frequency(id: BoardId) -> u64 {
        match id {
            BoardId::Virtual => sim_mcu::bus::CPU_HZ,
            // Pico SDK стандартная конфигурация 125 МГц после инициализации XOSC/PLL.
            BoardId::Rp2040 => 125_000_000,
            // RM0008, RCC_CR после сброса: HSI 8 МГц.
            BoardId::Stm32f103c8 => 8_000_000,
            // Pinout/схемы Arduino Uno/Nano: внешний резонатор 16 МГц.
            BoardId::Atmega328p => 16_000_000,
            // ESP32-C3 TRM главы 4, 7: профиль CPU/APB 80 МГц.
            BoardId::Esp32C3 => 80_000_000,
        }
    }

    pub fn restart(&mut self, watchdog: bool) {
        if self.rtc_cpu_restart(watchdog) {
            self.sleep_after_cpu_reset();
            return;
        }
        let mut reset = Self::new(self.id);
        reset.cycles = self.cycles;
        reset.adc.volts = self.adc.volts;
        reset.uart.done = std::mem::take(&mut self.uart.done);
        reset.watchdog_resets = self.watchdog_resets.saturating_add(u32::from(watchdog));
        reset.reset_events = std::mem::take(&mut self.reset_events);
        if watchdog {
            reset.reset_events.push(OutputEvent {
                cycle: self.cycles,
                output: BoardOutput::Reset(ResetCause::Watchdog),
            });
        }
        match (&mut self.chip, &mut reset.chip) {
            (Chip::Pico(old), Chip::Pico(new)) => {
                new.pins.keep_input(old.pins);
                new.watchdog_reason = if !watchdog {
                    0
                } else if old.forced_reset {
                    2
                } else {
                    1
                };
                new.retain_watchdog_scratch(old);
            }
            (Chip::Stm(old), Chip::Stm(new)) => {
                for (old, new) in old.ports.iter().zip(&mut new.ports) {
                    new.pins.keep_input(old.pins);
                }
                new.reset_flags = if watchdog { 1 << 29 } else { 1 << 26 };
            }
            (Chip::Avr(old), Chip::Avr(new)) => {
                for (old, new) in old.ports.iter().zip(&mut new.ports) {
                    new.keep_input(*old);
                }
                new.mcusr = if watchdog { 8 } else { 2 };
                if watchdog {
                    new.watchdog_after_reset(self.cycles);
                }
            }
            (Chip::Esp(old), Chip::Esp(new)) => {
                new.pins.keep_input(old.pins);
                old.buses.tick(self.cycles, true);
                new.buses.next_id = old.buses.next_id;
                new.buses.requests = std::mem::take(&mut old.buses.requests);
                new.buses.completed = std::mem::take(&mut old.buses.completed);
                if let Some(mut boot) = self.esp_boot.take() {
                    new.reset_boot_pads();
                    boot.reset();
                    reset.rtc = Some(Box::new(
                        esp_rtc::EspRtc::new(boot.wdt_delay_sel)
                            .expect("значение проверено при установке eFuse"),
                    ));
                    reset.esp_boot = Some(boot);
                }
            }
            _ => unreachable!(),
        }
        self.sleep_after_reset(&mut reset);
        self.rtc_finish_restart(&mut reset, watchdog);
        *self = reset;
    }

    pub fn access(&mut self, addr: u32, size: u8, v: Option<u32>) -> Access {
        self.access_with_memory(addr, size, v, None)
    }

    pub(crate) fn access_with_memory(
        &mut self,
        addr: u32,
        size: u8,
        v: Option<u32>,
        esp_memory: Option<&mut EspMemory>,
    ) -> Access {
        if let Some(value) = self.sleep_access(addr, size, v)? {
            return Ok(Some(value));
        }
        if let (Some(boot), Chip::Esp(chip)) = (&mut self.esp_boot, &mut self.chip)
            && let Some(value) =
                boot.access(chip, self.rtc.as_deref(), self.cycles, addr, size, v)?
        {
            return Ok(Some(value));
        }
        if let Chip::Esp(chip) = &mut self.chip
            && let Some(boot) = &self.esp_boot
            && let Some(value) = chip.dma.access(boot.pc, addr, size, v)?
        {
            return Ok(Some(value));
        }
        if let Some(value) = self.rtc_access(addr, size, v)? {
            return Ok(Some(value));
        }
        let now = self.cycles;
        if matches!(self.chip, Chip::Pico(_) | Chip::Stm(_))
            && (0xe000_e000..0xe001_0000).contains(&addr)
        {
            return self.nvic.access(
                self.id,
                addr,
                size,
                v,
                &mut self.vector_base,
                self.arm_active,
                self.arm_active_mask,
                &mut self.reset_requested,
            );
        }
        match &mut self.chip {
            Chip::Pico(p) if size == 4 => p.access(addr, v, now, &mut self.uart, &mut self.adc),
            Chip::Stm(p) if size == 2 || size == 4 => {
                p.access(addr, v, now, &mut self.uart, &mut self.adc)
            }
            Chip::Avr(p) if size == 1 => p.access(addr, v, now, &mut self.uart, &mut self.adc),
            Chip::Esp(p) if size == 4 => p.access(
                addr,
                v,
                now,
                &mut self.uart,
                &mut self.adc,
                self.esp_boot.as_deref(),
                esp_memory,
            ),
            _ => Ok(None),
        }
    }

    pub fn tick(&mut self, passed: u64) {
        self.tick_with_memory(passed, None);
    }

    pub(crate) fn tick_with_memory(&mut self, passed: u64, esp_memory: Option<&mut EspMemory>) {
        self.cycles = self.cycles.saturating_add(passed);
        self.uart.tick(self.cycles);
        let adc_completed = self.adc.tick(self.cycles);
        let watchdog = match &mut self.chip {
            Chip::Pico(p) => p.tick(passed, self.cycles),
            Chip::Stm(p) => p.tick(passed, self.cycles),
            Chip::Avr(p) => {
                if adc_completed {
                    p.adc_complete(self.adc.result);
                }
                p.tick(passed, self.cycles)
            }
            Chip::Esp(p) => p.tick(passed, self.cycles, self.esp_boot.as_deref(), esp_memory),
        };
        self.reset_requested |= watchdog;
        if watchdog {
            self.nvic.watchdog_reset = true;
        }
        if self.esp_boot.is_none() {
            self.tick_rtc(passed);
        }
        if matches!(self.chip, Chip::Pico(_) | Chip::Stm(_)) {
            self.nvic.tick(self.id, passed);
        }
    }

    pub fn watchdog_reset(&self) -> bool {
        self.nvic.watchdog_reset
    }

    pub fn arm_exception(&mut self, masked: bool) -> Option<u16> {
        let irqs = match &self.chip {
            Chip::Pico(p) => p.interrupts(),
            Chip::Stm(p) => p.interrupts(),
            _ => 0,
        };
        // Уровневый источник остаётся активным до очистки в обработчике.
        // Его повторно учитываем только при выходе из активного исключения.
        self.nvic.pending |= irqs & !(self.arm_active_mask >> 16);
        if masked {
            return None;
        }
        self.nvic.next(self.arm_active)
    }

    pub fn avr_irq(&mut self) -> Option<u32> {
        match &mut self.chip {
            Chip::Avr(p) => p.interrupt(),
            _ => None,
        }
    }

    pub(crate) fn install_esp_boot(
        &mut self,
        flash: esp_flash::EspFlash,
        phase: Option<esp_system::SystimerPhase>,
        wdt_delay_sel: u8,
        profile: Option<crate::EspResetProfile>,
    ) -> Result<(), BoardFault> {
        self.rtc = Some(Box::new(esp_rtc::EspRtc::new(wdt_delay_sel)?));
        self.esp_sleep = Some(Box::default());
        if let Chip::Esp(chip) = &mut self.chip {
            chip.reset_boot_pads();
        }
        self.esp_boot = Some(Box::new(esp::BootPeripherals::new(
            flash,
            phase,
            wdt_delay_sel,
            profile,
        )));
        Ok(())
    }

    pub(crate) fn enter_esp_flash_boot(&mut self) -> Result<(), BoardFault> {
        self.rtc
            .as_mut()
            .expect("RTC установлен вместе с flash")
            .enter_flash_boot()?;
        self.esp_boot.as_mut().expect("установлена flash").timg[0].enter_flash_boot();
        Ok(())
    }

    pub(crate) fn rtc_interrupt_pending(&self) -> Option<bool> {
        self.rtc
            .as_deref()
            .map(|rtc| rtc.interrupt_pending() || self.sleep_interrupt())
    }

    pub(crate) fn advance_esp_boot(
        &mut self,
        passed: u64,
        rates: crate::esp_clock::EspClockRates,
        stalled: bool,
    ) -> Result<(), BoardFault> {
        let boot = self.esp_boot.as_mut().expect("установлена flash");
        let before = boot.system.elapsed_quarter_ns();
        boot.system.advance_cpu(passed, rates, stalled)?;
        if let Chip::Esp(chip) = &mut self.chip {
            chip.advance_boot_io(boot, rates, before, passed, self.cycles, &mut self.uart)?;
        }
        let reset = boot.advance_timg(boot.system.elapsed_quarter_ns() - before, rates);
        if let Some((group, action)) = reset {
            self.request_mwdt_reset(group, action);
        }
        self.advance_awake_rtc(passed, rates.cpu_cycle_quarters());
        Ok(())
    }

    pub fn feed_watchdog(&mut self) {
        if let Chip::Avr(p) = &mut self.chip {
            p.feed(self.cycles);
        }
    }

    pub fn input(&mut self, pin: u32, high: bool) {
        let location = match &mut self.chip {
            Chip::Pico(p) if pin < 30 => Some((&mut p.pins, pin)),
            Chip::Esp(p) if pin < 22 => Some((&mut p.pins, pin)),
            Chip::Stm(p) if pin < 48 => Some((&mut p.ports[(pin / 16) as usize].pins, pin % 16)),
            Chip::Avr(p) => avr::pin_location(pin).map(|(port, bit)| (&mut p.ports[port], bit)),
            _ => None,
        };
        if let Some((pins, bit)) = location {
            let mask = 1 << bit;
            let changed = pins.driven & mask == 0 || (pins.input & mask != 0) != high;
            pins.driven |= mask;
            if high {
                pins.input |= mask;
            } else {
                pins.input &= !mask;
            }
            if changed {
                self.sleep_pin_changed(pin);
            }
        }
    }

    pub fn set_adc(&mut self, channel: u32, volts: f64) {
        if let Some(entry) = self.adc.volts.get_mut(channel as usize) {
            *entry = Some(volts);
        }
    }

    pub fn disconnect_adc(&mut self, channel: u32) {
        if let Some(slot) = self.adc.volts.get_mut(channel as usize) {
            *slot = None;
        }
    }

    pub fn output(&self, pin: u32) -> Option<bool> {
        if self.sleeping_mode().is_some() {
            return match self.gpio_mode(pin)? {
                GpioMode::Output(high) => Some(high),
                _ => None,
            };
        }
        match &self.chip {
            Chip::Pico(p) => p.output(pin),
            Chip::Stm(p) => p.output(pin),
            Chip::Avr(p) => p.output(pin),
            Chip::Esp(p) => p.output(pin),
        }
    }

    pub fn gpio_mode(&self, pin: u32) -> Option<GpioMode> {
        if !crate::pin_spec(self.id, pin).is_some_and(|spec| spec.gpio) {
            return None;
        }
        if let (Chip::Esp(chip), Some(mode)) = (&self.chip, self.sleeping_mode()) {
            return chip.sleep_pad_mode(pin, mode);
        }
        if let Chip::Avr(cpu) = &self.chip {
            return cpu.gpio_mode(pin);
        }
        if let Some(high) = self.output(pin) {
            return Some(GpioMode::Output(high));
        }
        if let Chip::Esp(chip) = &self.chip
            && self.esp_boot.is_some()
            && chip.pad_is_flash(pin)
        {
            // EspFlash моделирует транзакции, уровни физической шины открыты.
            return None;
        }
        match &self.chip {
            Chip::Pico(p) => p.passive_pin(pin),
            Chip::Stm(p) => p.passive_pin(pin),
            Chip::Avr(_) => unreachable!(),
            Chip::Esp(p) => p.passive_pin(pin),
        }
    }

    pub fn pwm(&self, pin: u32) -> Option<f64> {
        if self.sleeping_mode().is_some() {
            return None;
        }
        match &self.chip {
            Chip::Pico(p) => p.pwm(pin),
            Chip::Stm(p) => p.pwm(pin),
            Chip::Avr(p) => p.pwm(pin),
            Chip::Esp(p) => p.pwm(pin).map(|pwm| pwm.duty),
        }
    }

    pub fn pwm_output(&self, pin: u32) -> Option<PwmOutput> {
        if self.sleeping_mode().is_some() {
            return None;
        }
        if let Chip::Esp(p) = &self.chip {
            return p.pwm(pin);
        }
        let period = match &self.chip {
            Chip::Pico(p) => p.pwm_period(pin)?,
            Chip::Stm(p) => p.pwm_period(pin)?,
            Chip::Avr(p) => p.pwm_period(pin)?,
            Chip::Esp(_) => unreachable!(),
        };
        Some(PwmOutput::new(self.pwm(pin)?, period))
    }

    pub fn take_uart_events(&mut self) -> Vec<OutputEvent> {
        std::mem::take(&mut self.uart.done)
    }

    pub fn has_bus_request(&self) -> bool {
        matches!(&self.chip, Chip::Esp(p) if !p.buses.requests.is_empty())
    }

    pub fn take_bus_requests(&mut self) -> Vec<OutputEvent> {
        match &mut self.chip {
            Chip::Esp(p) => std::mem::take(&mut p.buses.requests),
            _ => Vec::new(),
        }
    }

    pub fn take_bus_completed(&mut self) -> Vec<OutputEvent> {
        match &mut self.chip {
            Chip::Esp(p) => std::mem::take(&mut p.buses.completed),
            _ => Vec::new(),
        }
    }

    pub fn bus_fault(&self) -> Option<&BoardFault> {
        match &self.chip {
            Chip::Esp(p) => p.buses.fault.as_ref(),
            _ => None,
        }
    }

    pub fn take_reset_events(&mut self) -> Vec<OutputEvent> {
        std::mem::take(&mut self.reset_events)
    }

    pub fn cancel_bus(&mut self) {
        if let Chip::Esp(p) = &mut self.chip {
            p.buses.tick(self.cycles, true);
        }
    }

    pub fn next_bus_id(&mut self, id: Option<u64>) -> u64 {
        if let Chip::Esp(p) = &mut self.chip {
            if let Some(id) = id {
                p.buses.next_id = id;
            }
            p.buses.next_id
        } else {
            0
        }
    }

    pub fn apply_port(&mut self, input: BoardInput) -> Result<InputOutcome, BoardFault> {
        if self.sleeping_mode().is_some() {
            return Err(unsupported("пакетный порт во время сна ESP32-C3"));
        }
        match (&mut self.chip, input) {
            (Chip::Esp(p), BoardInput::Uart { port, bytes }) => {
                p.uart_receive(port, self.cycles, &bytes, self.esp_boot.as_deref())
            }
            (Chip::Esp(p), input @ (BoardInput::I2cReply { .. } | BoardInput::SpiReply { .. })) => {
                p.buses.reply(self.cycles, input)?;
                Ok(InputOutcome::Applied)
            }
            _ => Err(unsupported("входной порт в этом профиле")),
        }
    }
}

// PM0056 §4.3–4.5; RP2040 Datasheet §2.4.
#[derive(Clone, Default)]
struct Nvic {
    enabled: u64,
    pending: u64,
    priority: [u32; 16],
    system: [u32; 3],
    ctrl: u32,
    reload: u32,
    val: u32,
    flag: bool,
    systick_pending: bool,
    pendsv: bool,
    phase: u64,
    watchdog_reset: bool,
}

impl Nvic {
    // Регистр SCB меняет состояние активного исключения и запроса сброса машины.
    #[allow(clippy::too_many_arguments)]
    fn access(
        &mut self,
        id: BoardId,
        address: u32,
        size: u8,
        v: Option<u32>,
        vtor: &mut u32,
        active: u16,
        active_mask: u64,
        reset: &mut bool,
    ) -> Access {
        if size != 4 {
            return Ok(None);
        }
        let addr = address & 0xfff;
        let high = if id == BoardId::Rp2040 { 0 } else { 4 };
        let width_mask = if id == BoardId::Rp2040 {
            (1u64 << 26) - 1
        } else {
            (1u64 << 43) - 1
        };
        let result = match addr {
            0x010 => {
                if let Some(v) = v {
                    self.ctrl = (v & 7) | if id == BoardId::Rp2040 { 4 } else { 0 };
                    0
                } else {
                    let flag = std::mem::take(&mut self.flag);
                    self.ctrl | (u32::from(flag) << 16)
                }
            }
            0x014 => return rw(v, &mut self.reload, 0xff_ffff),
            0x018 => {
                if v.is_some() {
                    self.val = 0;
                    self.flag = false;
                    0
                } else {
                    self.val
                }
            }
            0x100..=0x104 | 0x180..=0x184 | 0x200..=0x204 | 0x280..=0x284 if addr & 4 <= high => {
                let shift = (addr & 4) * 8;
                let reg = if addr & 0x200 != 0 {
                    &mut self.pending
                } else {
                    &mut self.enabled
                };
                if let Some(v) = v {
                    let bits = (u64::from(v) << shift) & width_mask;
                    if addr & 0x80 != 0 {
                        *reg &= !bits;
                    } else {
                        *reg |= bits;
                    }
                    0
                } else {
                    (*reg >> shift) as u32
                }
            }
            0x300..=0x304 if id == BoardId::Stm32f103c8 => {
                return ro(v, ((active_mask >> 16) >> ((addr & 4) * 8)) as u32);
            }
            0x400..=0x428 if id != BoardId::Rp2040 || addr <= 0x418 => {
                let mask = if id == BoardId::Rp2040 {
                    0xc0c0_c0c0
                } else {
                    0xf0f0_f0f0
                };
                return rw(v, &mut self.priority[((addr - 0x400) / 4) as usize], mask);
            }
            0xd00 => {
                return ro(
                    v,
                    if id == BoardId::Rp2040 {
                        0x410c_c601
                    } else {
                        0x411f_c231
                    },
                );
            }
            0xd04 => {
                if let Some(v) = v {
                    if v & !0x1e00_0000 != 0 {
                        return Err(unsupported("SCB ICSR"));
                    }
                    if v & 1 << 28 != 0 {
                        self.pendsv = true;
                    }
                    if v & 1 << 27 != 0 {
                        self.pendsv = false;
                    }
                    if v & 1 << 26 != 0 {
                        self.systick_pending = true;
                    }
                    if v & 1 << 25 != 0 {
                        self.systick_pending = false;
                    }
                    0
                } else {
                    u32::from(active)
                        | (u32::from(self.pendsv) << 28)
                        | (u32::from(self.systick_pending) << 26)
                }
            }
            0xd08 => return rw(v, vtor, 0xffff_ff00),
            0xd0c => {
                if let Some(v) = v {
                    if v >> 16 != 0x05fa || v & 0xffff & !4 != 0 {
                        return Err(unsupported("SCB AIRCR"));
                    }
                    *reset |= v & 4 != 0;
                    0
                } else {
                    0xfa05_0000
                }
            }
            0xd18..=0xd20 if id != BoardId::Rp2040 || addr != 0xd18 => {
                let mask = if id == BoardId::Rp2040 {
                    0xc0c0_c0c0
                } else {
                    0xf0f0_f0f0
                };
                return rw(v, &mut self.system[((addr - 0xd18) / 4) as usize], mask);
            }
            _ => return Ok(None),
        };
        Ok(Some(result))
    }

    fn tick(&mut self, id: BoardId, passed: u64) {
        if self.ctrl & 1 == 0 {
            return;
        }
        let div = if id == BoardId::Rp2040 || self.ctrl & 4 != 0 {
            1
        } else {
            8
        };
        let count = passed + self.phase;
        let ticks = (count / div) as u32;
        self.phase = count % div;
        if ticks == 0 || self.reload == 0 {
            return;
        }
        let until = if self.val == 0 {
            self.reload + 1
        } else {
            self.val
        };
        if ticks >= until {
            self.flag = true;
            self.systick_pending |= self.ctrl & 2 != 0;
            let rest = (ticks - until) % (self.reload + 1);
            self.val = (self.reload + 1 - rest) % (self.reload + 1);
        } else {
            self.val = until - ticks;
        }
    }

    fn next(&mut self, active: u16) -> Option<u16> {
        let mut bits = self.enabled & self.pending;
        let priority = |exception: u16| {
            if exception >= 16 {
                let irq = (exception - 16) as usize;
                (self.priority[irq / 4] >> (8 * (irq % 4))) as u8
            } else {
                let i = exception.saturating_sub(4) as usize;
                (self.system[i / 4] >> (8 * (i % 4))) as u8
            }
        };
        let mut best = None;
        if self.pendsv {
            best = Some((priority(14), 14));
        }
        if self.systick_pending {
            best = best.into_iter().chain([(priority(15), 15)]).min();
        }
        while bits != 0 {
            let irq = bits.trailing_zeros() as u16;
            bits &= bits - 1;
            best = best
                .into_iter()
                .chain([(priority(irq + 16), irq + 16)])
                .min();
        }
        let (level, vector) = best?;
        if active != 0 && level >= priority(active) {
            return None;
        }
        match vector {
            14 => self.pendsv = false,
            15 => self.systick_pending = false,
            _ => self.pending &= !(1 << (vector - 16)),
        }
        Some(vector)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn write(hw: &mut Hardware, addr: u32, size: u8, value: u32) {
        assert_eq!(
            hw.access(addr, size, Some(value)).unwrap(),
            Some(0),
            "{addr:#x}"
        );
    }

    fn read(hw: &mut Hardware, addr: u32, size: u8) -> u32 {
        hw.access(addr, size, None)
            .unwrap()
            .expect("регистр реализован")
    }

    #[test]
    fn tim1_stm_подключает_pwm_после_moe() {
        let mut hw = Hardware::new(BoardId::Stm32f103c8);
        // RM0008: TIM1EN и IOPAEN. PA8 в режиме AF push-pull 2 МГц.
        write(&mut hw, 0x4002_1018, 4, 0x804);
        write(&mut hw, 0x4001_0804, 4, 0x4444_444a);
        let tim = 0x4001_2c00;
        write(&mut hw, tim + 0x2c, 4, 99);
        write(&mut hw, tim + 0x34, 4, 25);
        write(&mut hw, tim + 0x18, 4, 0x60);
        write(&mut hw, tim + 0x20, 4, 1);
        write(&mut hw, tim, 4, 1);
        assert_eq!(hw.pwm(8), None);
        write(&mut hw, tim + 0x44, 4, 1 << 15);
        assert_eq!(hw.pwm(8), Some(0.25));
        hw.tick(24);
        assert_eq!(hw.output(8), Some(true));
        hw.tick(1);
        assert_eq!(hw.output(8), Some(false));
    }

    #[test]
    fn timer1_avr_соблюдает_ширину_ocr_и_fast_pwm() {
        let mut hw = Hardware::new(BoardId::Atmega328p);
        // 7810D Timer1: OC1A на PB1, WGM=7, TOP=1023, OCR1A=255.
        write(&mut hw, 0x24, 1, 2);
        write(&mut hw, 0x89, 1, 0);
        write(&mut hw, 0x88, 1, 255);
        write(&mut hw, 0x80, 1, 0x83);
        write(&mut hw, 0x81, 1, 0x09);
        assert_eq!(hw.pwm(9), Some(0.25));
        hw.tick(255);
        assert_eq!(hw.output(9), Some(true));
        hw.tick(1);
        assert_eq!(read(&mut hw, 0x84, 1), 0);
        assert_eq!(read(&mut hw, 0x85, 1), 1);
        assert_eq!(hw.output(9), Some(false));
    }

    #[test]
    fn avr_adif_очищается_отдельно_от_результата() {
        let mut hw = Hardware::new(BoardId::Atmega328p);
        hw.set_adc(0, 2.5);
        // 7810D: AVCC 5 В, первое преобразование 25 тактов АЦП, делитель 128.
        write(&mut hw, 0x7c, 1, 0x40);
        write(&mut hw, 0x7a, 1, 0xc7);
        hw.tick(25 * 128 - 1);
        assert_eq!(read(&mut hw, 0x7a, 1), 0xc7);
        hw.tick(1);
        assert_eq!(read(&mut hw, 0x7a, 1), 0x97);
        write(&mut hw, 0x7a, 1, 0x97);
        assert_eq!(read(&mut hw, 0x7a, 1), 0x87);
        assert_eq!(read(&mut hw, 0x78, 1), 0);
        assert_eq!(read(&mut hw, 0x79, 1), 2);
    }
}
