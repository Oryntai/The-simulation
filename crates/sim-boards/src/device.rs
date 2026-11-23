use crate::ports::{OutputPort, PortSample, PortValue, end_cycle, event_time};
use crate::{
    Advance, BoardInput, BoardOutput, Capabilities, InputEvent, InputOutcome, OutputEvent,
    PwmOutput, ResetCause, capabilities,
};
use crate::{
    BoardFault, BoardId, arm::Arm, avr::Avr, image, memory::Bus, peripheral::Hardware, riscv::Riscv,
};

/// Интерфейс исполнения прошивки. Успех run_for означает, что пройден
/// запрошенный интервал поддерживаемой модели.
pub trait BoardSim: Send {
    fn board(&self) -> BoardId;
    fn load(&mut self, image: &[u8]) -> Result<(), BoardFault>;
    fn reset(&mut self);
    fn run_for(&mut self, seconds: f64) -> Result<(), BoardFault>;
    fn set_pin_input(&mut self, pin: u32, high: bool);
    fn set_adc_volts(&mut self, channel: u32, volts: f64);
    fn pin_output(&self, pin: u32) -> Option<bool>;
    fn pwm_duty(&self, pin: u32) -> Option<f64>;
    fn uart_take(&mut self) -> Vec<u8>;

    fn clone_state(&self) -> Box<dyn BoardSim>;
    fn capabilities(&self) -> Capabilities {
        capabilities(self.board())
    }
    fn cycles(&self) -> u64;
    fn powered(&self) -> bool;
    fn reset_cause(&self) -> Option<ResetCause>;
    fn exit_code(&self) -> Option<u32> {
        None
    }
    fn watchdog_resets(&self) -> u32;
    fn write_config(&mut self, bytes: &[u8]) -> Result<(), BoardFault>;
    /// Продвигает не более заданного числа тактов. Может остановиться на запросе шины.
    fn run_cycles(&mut self, cycles: u64) -> Result<Advance, BoardFault>;
    fn apply(&mut self, event: InputEvent) -> Result<InputOutcome, BoardFault>;
    fn pwm(&self, port: u32) -> Option<PwmOutput>;
    fn take_events(&mut self) -> Vec<OutputEvent>;
    fn sample(&self, port: OutputPort) -> PortSample {
        let value = match port {
            OutputPort::Gpio(pin) => self.pin_output(pin).map(PortValue::Logic),
            OutputPort::Pwm(channel) => self.pwm(channel).map(PortValue::Pwm),
        };
        PortSample {
            cycle: self.cycles(),
            port,
            value,
        }
    }
}

impl Clone for Box<dyn BoardSim> {
    fn clone(&self) -> Self {
        self.clone_state()
    }
}

pub fn new_board(id: BoardId) -> Box<dyn BoardSim> {
    if id == BoardId::Virtual {
        return Box::new(crate::virtual_board::VirtualBoard::default());
    }
    Box::new(Device {
        id,
        machine: None,
        fault: None,
        digital: [None; 48],
        analog: [None; 18],
        powered: true,
        executing: true,
        image: None,
        cycle: 0,
        reset_cause: None,
        watchdog_resets: 0,
        next_bus_id: 0,
        events: Vec::new(),
    })
}

#[derive(Clone)]
enum Cpu {
    Arm(Arm),
    Avr(Avr),
    Riscv(Riscv),
}

impl Cpu {
    fn boot(bus: &mut Bus, entry: u32) -> Result<Self, BoardFault> {
        Ok(match bus.id {
            BoardId::Virtual => unreachable!("виртуальный профиль имеет собственное ядро"),
            BoardId::Rp2040 => Self::Arm(Arm::boot(bus, entry)?),
            BoardId::Stm32f103c8 => Self::Arm(Arm::boot(bus, 0)?),
            BoardId::Esp32C3 => Self::Riscv(Riscv::boot(bus, entry)?),
            BoardId::Atmega328p => {
                bus.fetch(0, 2)?;
                Self::Avr(Avr::default())
            }
        })
    }

    fn step(&mut self, bus: &mut Bus) -> Result<u64, BoardFault> {
        match self {
            Self::Arm(cpu) => cpu.step(bus),
            Self::Avr(cpu) => cpu.step(bus),
            Self::Riscv(cpu) => cpu.step(bus),
        }
    }
}

#[derive(Clone)]
struct Machine {
    bus: Bus,
    cpu: Cpu,
    entry: u32,
    remaining: u64,
    fractional: f64,
}

impl Machine {
    fn restart(&mut self, watchdog: bool) -> Result<(), BoardFault> {
        self.bus.hw.restart(watchdog);
        if self.bus.id == BoardId::Rp2040 {
            self.bus.hw.vector_base = self.entry;
        }
        self.cpu = Cpu::boot(&mut self.bus, self.entry)?;
        Ok(())
    }

    fn run(&mut self, cycles: u64, executing: bool) -> Result<(), BoardFault> {
        let mut todo = cycles;
        while todo != 0 && !self.bus.hw.has_bus_request() {
            if !executing {
                self.bus.hw.tick(1);
                todo -= 1;
            } else {
                if self.remaining == 0 {
                    self.remaining = self.cpu.step(&mut self.bus)?;
                }
                let spend = self.remaining.min(todo);
                self.bus.hw.tick(spend);
                self.remaining -= spend;
                todo -= spend;
            }
            if let Some(fault) = self.bus.hw.bus_fault() {
                return Err(fault.clone());
            }
            if self.bus.hw.reset_requested {
                let watchdog = self.bus.hw.watchdog_reset();
                self.restart(watchdog)?;
                if !watchdog {
                    let hw = &mut self.bus.hw;
                    hw.reset_events.push(OutputEvent {
                        cycle: hw.cycles,
                        output: BoardOutput::Reset(ResetCause::Software),
                    });
                }
                self.remaining = 0;
            }
        }
        Ok(())
    }
}

#[derive(Clone)]
struct Device {
    id: BoardId,
    machine: Option<Machine>,
    fault: Option<BoardFault>,
    digital: [Option<bool>; 48],
    analog: [Option<f64>; 18],
    powered: bool,
    executing: bool,
    image: Option<std::sync::Arc<[u8]>>,
    cycle: u64,
    reset_cause: Option<ResetCause>,
    watchdog_resets: u32,
    next_bus_id: u64,
    events: Vec<OutputEvent>,
}

impl Device {
    fn emit(&mut self, output: BoardOutput) {
        self.events.push(OutputEvent {
            cycle: self.cycle,
            output,
        });
    }

    fn disconnect(&mut self) {
        if let Some(mut machine) = self.machine.take() {
            let hw = &mut machine.bus.hw;
            hw.cancel_bus();
            self.events.extend(hw.take_uart_events());
            self.events.extend(hw.take_bus_requests());
            self.events.extend(hw.take_bus_completed());
            self.events.extend(hw.take_reset_events());
            self.next_bus_id = hw.next_bus_id(None);
        }
    }

    fn set_power(&mut self, on: bool) -> Result<(), BoardFault> {
        if self.powered == on {
            return Ok(());
        }
        self.powered = on;
        self.emit(BoardOutput::Power { on });
        if !on {
            self.disconnect();
        } else if let Some(image) = self.image.clone() {
            self.load(&image)?;
            self.emit(BoardOutput::Reset(ResetCause::PowerOn));
        }
        Ok(())
    }
}

pub(crate) fn take_uart_events(events: &mut Vec<OutputEvent>) -> Vec<u8> {
    events.sort_by_key(|event| event.cycle);
    let mut bytes = Vec::new();
    events.retain(|event| {
        if let BoardOutput::Uart { byte, .. } = event.output {
            bytes.push(byte);
            false
        } else {
            true
        }
    });
    bytes
}

impl BoardSim for Device {
    fn board(&self) -> BoardId {
        self.id
    }

    fn load(&mut self, data: &[u8]) -> Result<(), BoardFault> {
        // Не оставляем прежнюю прошивку доступной для запуска после неудачной записи.
        self.disconnect();
        self.image = None;
        self.reset_cause = None;
        if !self.powered {
            return Err(BoardFault::Unpowered);
        }
        let loaded: Result<Machine, BoardFault> = (|| {
            let arch = match self.id {
                BoardId::Virtual => unreachable!("виртуальный профиль имеет собственный загрузчик"),
                BoardId::Rp2040 | BoardId::Stm32f103c8 => image::Machine::Arm,
                BoardId::Esp32C3 => image::Machine::RiscV,
                BoardId::Atmega328p => image::Machine::Avr,
            };
            let image = if arch == image::Machine::Avr && !image::is_elf(data) {
                image::parse_ihex(data)?
            } else {
                image::parse_elf(data, arch)?
            };
            let mut bus = Bus::new(self.id, &image)?;
            bus.hw.cycles = self.cycle;
            bus.hw.next_bus_id(Some(self.next_bus_id));
            let entry = match self.id {
                BoardId::Rp2040 => {
                    // Pico SDK: 256 байт boot2, затем таблица векторов. Поддержан и ELF
                    // стенда с таблицей векторов непосредственно в начале XIP.
                    let at = 0x1000_0000;
                    let mut plausible = [at, at + 0x100]
                        .into_iter()
                        .filter(|&base| Arm::boot(&mut bus, base).is_ok());
                    let vector = plausible
                        .next()
                        .filter(|_| plausible.next().is_none())
                        .ok_or_else(|| {
                            BoardFault::BadImage(
                                "таблица векторов Pico отсутствует или неоднозначна".into(),
                            )
                        })?;
                    bus.hw.vector_base = vector;
                    vector
                }
                BoardId::Esp32C3 => image
                    .entry
                    .ok_or_else(|| BoardFault::BadImage("нет точки входа".into()))?,
                _ => 0,
            };
            let cpu = Cpu::boot(&mut bus, entry)?;
            for (pin, value) in self.digital.iter().enumerate() {
                if let Some(high) = value {
                    bus.hw.input(pin as u32, *high);
                }
            }
            for (channel, value) in self.analog.iter().enumerate() {
                if let Some(volts) = value {
                    bus.hw.set_adc(channel as u32, *volts);
                }
            }
            Ok(Machine {
                bus,
                cpu,
                entry,
                remaining: 0,
                fractional: 0.0,
            })
        })();
        match loaded {
            Ok(machine) => {
                self.machine = Some(machine);
                self.image = Some(data.into());
                self.reset_cause = Some(ResetCause::PowerOn);
                self.watchdog_resets = 0;
                self.fault = None;
                Ok(())
            }
            Err(e) => {
                self.fault = Some(e.clone());
                Err(e)
            }
        }
    }

    fn reset(&mut self) {
        if self.powered
            && let Some(machine) = self.machine.as_mut()
        {
            self.fault = machine.restart(false).err();
            self.reset_cause = Some(ResetCause::Host);
            self.events.push(OutputEvent {
                cycle: self.cycle,
                output: BoardOutput::Reset(ResetCause::Host),
            });
            machine.remaining = 0;
            machine.fractional = 0.0;
        }
    }

    fn run_for(&mut self, seconds: f64) -> Result<(), BoardFault> {
        if !self.powered {
            return Err(BoardFault::Unpowered);
        }
        let result = (|| {
            if let Some(fault) = &self.fault {
                return Err(fault.clone());
            }
            if !seconds.is_finite() || seconds < 0.0 {
                return Err(BoardFault::Misconfigured(
                    "длительность должна быть конечной и неотрицательной".into(),
                ));
            }
            let machine = self.machine.as_mut().ok_or(BoardFault::NoImage)?;
            let ticks = seconds * Hardware::frequency(self.id) as f64 + machine.fractional;
            // 2^53 — граница точного целочисленного представления f64.
            if !ticks.is_finite()
                || ticks >= (1u64 << 53) as f64
                || machine
                    .bus
                    .hw
                    .cycles
                    .checked_add(ticks as u64)
                    .is_none_or(|n| n >= 1u64 << 53)
            {
                return Err(BoardFault::Misconfigured(
                    "интервал вне точного диапазона счётчика тактов".into(),
                ));
            }
            machine.fractional = ticks.fract();
            let target = self.cycle + ticks as u64;
            let mut state = self.run_cycles(ticks as u64)?;
            while state.bus_request_pending {
                if let Some(machine) = &mut self.machine {
                    self.events.extend(machine.bus.hw.take_bus_requests());
                }
                state = self.run_cycles(target - self.cycle)?;
            }
            Ok(())
        })();
        if let Err(e) = &result {
            self.fault = Some(e.clone());
        }
        result
    }

    fn set_pin_input(&mut self, pin: u32, high: bool) {
        if crate::pin_spec(self.id, pin).is_some_and(|p| p.gpio)
            && let Some(value) = self.digital.get_mut(pin as usize)
        {
            *value = Some(high);
            if let Some(machine) = &mut self.machine {
                machine.bus.hw.input(pin, high);
            }
        } else {
            self.fault = Some(BoardFault::Misconfigured(
                "недопустимый номер цифрового входа".into(),
            ));
        }
    }

    fn set_adc_volts(&mut self, channel: u32, volts: f64) {
        let channels = self.capabilities().adc_channels;
        if channel < channels
            && let Some(value) = self.analog.get_mut(channel as usize)
        {
            *value = Some(volts);
            if let Some(machine) = &mut self.machine {
                machine.bus.hw.set_adc(channel, volts);
            }
        } else {
            self.fault = Some(BoardFault::Misconfigured(
                "недопустимый номер канала АЦП".into(),
            ));
        }
        if !volts.is_finite() {
            self.fault = Some(BoardFault::Misconfigured(
                "задано неконечное напряжение АЦП".into(),
            ));
        }
    }

    fn pin_output(&self, pin: u32) -> Option<bool> {
        if !self.powered || self.fault.is_some() {
            return None;
        }
        self.machine.as_ref()?.bus.hw.output(pin)
    }

    fn pwm_duty(&self, pin: u32) -> Option<f64> {
        if !self.powered || self.fault.is_some() {
            return None;
        }
        self.machine
            .as_ref()?
            .bus
            .hw
            .pwm(pin)
            .filter(|v| v.is_finite())
    }

    fn uart_take(&mut self) -> Vec<u8> {
        if let Some(machine) = &mut self.machine {
            self.events.extend(machine.bus.hw.take_uart_events());
        }
        take_uart_events(&mut self.events)
    }

    fn clone_state(&self) -> Box<dyn BoardSim> {
        Box::new(self.clone())
    }

    fn cycles(&self) -> u64 {
        self.cycle
    }

    fn powered(&self) -> bool {
        self.powered
    }

    fn reset_cause(&self) -> Option<ResetCause> {
        self.reset_cause
    }

    fn watchdog_resets(&self) -> u32 {
        self.watchdog_resets
    }

    fn write_config(&mut self, bytes: &[u8]) -> Result<(), BoardFault> {
        if bytes.is_empty() {
            return Ok(());
        }
        Err(BoardFault::Unsupported(
            "у профиля нет блока конфигурации".into(),
        ))
    }

    fn run_cycles(&mut self, count: u64) -> Result<Advance, BoardFault> {
        let target = end_cycle(self.cycle, count)?;
        if !self.powered {
            self.cycle = target;
            return Ok(Advance {
                cycle: target,
                bus_request_pending: false,
            });
        }
        if let Some(fault) = &self.fault {
            return Err(fault.clone());
        }
        if self.analog.iter().flatten().any(|v| !v.is_finite()) {
            return Err(BoardFault::Misconfigured(
                "задано неконечное напряжение АЦП".into(),
            ));
        }
        let machine = self.machine.as_mut().ok_or(BoardFault::NoImage)?;
        let result = machine.run(count, self.executing);
        let hw = &mut machine.bus.hw;
        self.cycle = hw.cycles;
        self.watchdog_resets = hw.watchdog_resets;
        let resets = hw.take_reset_events();
        if let Some(OutputEvent {
            output: BoardOutput::Reset(cause),
            ..
        }) = resets.last()
        {
            self.reset_cause = Some(*cause);
        }
        self.events.extend(hw.take_bus_completed());
        self.events.extend(resets);
        let bus_request_pending = hw.has_bus_request();
        if let Err(fault) = result {
            self.fault = Some(fault.clone());
            self.emit(BoardOutput::Fault(fault.clone()));
            return Err(fault);
        }
        Ok(Advance {
            cycle: self.cycle,
            bus_request_pending,
        })
    }

    fn apply(&mut self, event: InputEvent) -> Result<InputOutcome, BoardFault> {
        event_time(self.cycle, event.cycle)?;
        match event.input {
            BoardInput::Power { on } => {
                if self.powered == on {
                    return Ok(InputOutcome::Unchanged);
                }
                self.set_power(on)?;
            }
            BoardInput::Execution { enabled } => {
                if self.executing == enabled {
                    return Ok(InputOutcome::Unchanged);
                }
                self.executing = enabled;
                self.emit(BoardOutput::Execution { enabled });
            }
            BoardInput::Gpio { pin, high }
                if crate::pin_spec(self.id, pin).is_some_and(|p| p.gpio) =>
            {
                self.set_pin_input(pin, high);
            }
            BoardInput::Adc { channel, volts }
                if channel < self.capabilities().adc_channels && volts.is_finite() =>
            {
                self.set_adc_volts(channel, volts);
            }
            BoardInput::DisconnectAdc { channel } if channel < self.capabilities().adc_channels => {
                self.analog[channel as usize] = None;
                if let Some(machine) = &mut self.machine {
                    machine.bus.hw.disconnect_adc(channel);
                }
            }
            BoardInput::Gpio { .. } | BoardInput::Adc { .. } | BoardInput::DisconnectAdc { .. } => {
                return Err(BoardFault::Misconfigured(
                    "неверный вход GPIO или АЦП".into(),
                ));
            }
            input => {
                if !self.powered {
                    return Err(BoardFault::Unpowered);
                }
                if let Some(fault) = &self.fault {
                    return Err(fault.clone());
                }
                let machine = self.machine.as_mut().ok_or(BoardFault::NoImage)?;
                return machine.bus.hw.apply_port(input);
            }
        }
        Ok(InputOutcome::Applied)
    }

    fn pwm(&self, port: u32) -> Option<PwmOutput> {
        if !self.powered || self.fault.is_some() {
            return None;
        }
        self.machine.as_ref()?.bus.hw.pwm_output(port)
    }

    fn take_events(&mut self) -> Vec<OutputEvent> {
        if let Some(machine) = &mut self.machine {
            let hw = &mut machine.bus.hw;
            self.events.extend(hw.take_uart_events());
            self.events.extend(hw.take_bus_requests());
            self.events.extend(hw.take_bus_completed());
            self.events.extend(hw.take_reset_events());
        }
        self.events.sort_by_key(|event| event.cycle);
        std::mem::take(&mut self.events)
    }
}
