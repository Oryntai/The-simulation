use sim_mcu::{HaltReason, Mcu, ResetReason};

use crate::ports::{end_cycle, event_time};
use crate::{
    Advance, BoardFault, BoardId, BoardInput, BoardOutput, BoardSim, GpioMode, InputEvent,
    InputOutcome, OutputEvent, Packet, PwmOutput, ResetCause,
};

#[derive(Clone)]
pub(crate) struct VirtualBoard {
    mcu: Option<Mcu>,
    cycle: u64,
    target: f64,
    powered: bool,
    brownout: bool,
    executing: bool,
    fault: Option<BoardFault>,
    analog_invalid: [bool; sim_mcu::periph::adc::CHANNELS],
    analog: [Option<f64>; sim_mcu::periph::adc::CHANNELS],
    digital: [Option<bool>; 32], // GPIO.DIR и GPIO.OUT — 32-битные регистры ABI.
    events: Vec<OutputEvent>,
    halt_reported: bool,
    reset: Option<ResetCause>,
}

impl Default for VirtualBoard {
    fn default() -> Self {
        Self {
            mcu: None,
            cycle: 0,
            target: 0.0,
            powered: true,
            brownout: false,
            executing: true,
            fault: None,
            analog_invalid: [false; sim_mcu::periph::adc::CHANNELS],
            analog: [None; sim_mcu::periph::adc::CHANNELS],
            digital: [None; 32],
            events: Vec::new(),
            halt_reported: false,
            reset: None,
        }
    }
}

impl VirtualBoard {
    fn event(&mut self, output: BoardOutput) {
        self.events.push(OutputEvent {
            cycle: self.cycle,
            output,
        });
    }

    fn checked_mcu(&self) -> Result<&Mcu, BoardFault> {
        if let Some(fault) = &self.fault {
            return Err(fault.clone());
        }
        if self.analog_invalid.contains(&true) {
            return Err(BoardFault::Misconfigured(
                "неконечное напряжение АЦП".into(),
            ));
        }
        self.mcu.as_ref().ok_or(BoardFault::NoImage)
    }

    fn output_mcu(&self) -> Option<&Mcu> {
        let mcu = self.checked_mcu().ok()?;
        let stopped = mcu.halt_cycle().is_some_and(|cycle| cycle <= self.cycle);
        if !self.powered
            || stopped
                && matches!(
                    mcu.halt_reason(),
                    Some(
                        HaltReason::Breakpoint { .. }
                            | HaltReason::Fault(_)
                            | HaltReason::Exit(1..)
                    )
                )
        {
            return None;
        }
        Some(mcu)
    }

    fn set_power(&mut self, on: bool) {
        if self.powered != on {
            // Забираем только события, которые физически успели произойти.
            self.collect();
            self.powered = on;
            if !on {
                self.events.retain(|event| event.cycle <= self.cycle);
            }
            self.event(BoardOutput::Power { on });
            if on && self.mcu.is_some() {
                self.reset_mcu(ResetCause::PowerOn);
            }
        }
    }

    fn reset_mcu(&mut self, cause: ResetCause) {
        self.collect();
        if let Some(mcu) = &mut self.mcu {
            mcu.reset();
            self.fault = None;
            self.halt_reported = false;
            self.reset = Some(cause);
            self.event(BoardOutput::Reset(cause));
        }
    }

    fn collect(&mut self) {
        let Some(mcu) = &mut self.mcu else {
            return;
        };
        self.events.extend(
            mcu.take_timed_uart()
                .into_iter()
                .map(|(cycle, byte)| OutputEvent {
                    cycle,
                    output: BoardOutput::Uart { port: 0, byte },
                }),
        );
        self.events
            .extend(
                mcu.take_timed_packets()
                    .into_iter()
                    .map(|(cycle, bytes)| OutputEvent {
                        cycle,
                        output: BoardOutput::Radio {
                            port: 0,
                            packet: Packet {
                                bytes,
                                rssi_dbm: None,
                            },
                        },
                    }),
            );
        self.collect_state();
    }

    fn collect_state(&mut self) {
        let Some(mcu) = &mut self.mcu else {
            return;
        };
        for (cycle, reason) in mcu.take_timed_resets() {
            let cause = match reason {
                ResetReason::PowerOn => ResetCause::PowerOn,
                ResetReason::Watchdog => ResetCause::Watchdog,
                ResetReason::Software => ResetCause::Software,
            };
            self.reset = Some(cause);
            self.events.push(OutputEvent {
                cycle,
                output: BoardOutput::Reset(cause),
            });
        }
        self.events.extend(
            mcu.take_timed_faults()
                .into_iter()
                .map(|(cycle, fault)| OutputEvent {
                    cycle,
                    output: BoardOutput::Fault(BoardFault::Execution(fault.to_string())),
                }),
        );
        if !self.halt_reported
            && let (Some(cycle), Some(code)) = (mcu.halt_cycle(), mcu.exit_code())
            && cycle <= self.cycle
        {
            self.events.push(OutputEvent {
                cycle,
                output: BoardOutput::Halt { code },
            });
            self.halt_reported = true;
        }
    }
}

impl BoardSim for VirtualBoard {
    fn board(&self) -> BoardId {
        BoardId::Virtual
    }

    fn load(&mut self, image: &[u8]) -> Result<(), BoardFault> {
        self.mcu = None;
        self.halt_reported = false;
        self.reset = None;
        if !self.powered {
            return Err(BoardFault::Unpowered);
        }
        let mut mcu = Mcu::from_elf(image).map_err(|e| BoardFault::BadImage(e.to_string()))?;
        mcu.enable_host_timestamps();
        mcu.require_adc_inputs();
        mcu.advance_unpowered(self.cycle);
        for (pin, level) in self.digital.iter().enumerate() {
            if let Some(high) = level {
                mcu.set_gpio_input(pin, *high);
            }
        }
        for (channel, volts) in self.analog.iter().enumerate() {
            if let Some(volts) = volts {
                mcu.set_adc_volts(channel, *volts);
            }
        }
        self.target = self.cycle as f64;
        self.fault = None;
        self.reset = Some(ResetCause::PowerOn);
        self.mcu = Some(mcu);
        Ok(())
    }

    fn reset(&mut self) {
        if self.powered {
            self.reset_mcu(ResetCause::Host);
        }
    }

    fn run_for(&mut self, seconds: f64) -> Result<(), BoardFault> {
        if !self.powered {
            return Err(BoardFault::Unpowered);
        }
        let target = self.target + seconds * self.capabilities().cpu_hz as f64;
        if !seconds.is_finite() || seconds < 0.0 || target.round() >= (1u64 << 53) as f64 {
            return Err(BoardFault::Misconfigured(
                "недопустимая длительность шага".into(),
            ));
        }
        self.run_cycles(target.round() as u64 - self.cycle)?;
        self.target = target;
        Ok(())
    }

    fn set_pin_input(&mut self, pin: u32, high: bool) {
        if let Some(slot) = self.digital.get_mut(pin as usize) {
            *slot = Some(high);
            if let Some(mcu) = &mut self.mcu {
                mcu.set_gpio_input(pin as usize, high);
            }
        } else {
            self.fault = Some(BoardFault::Misconfigured("неверный вывод GPIO".into()));
        }
    }

    fn set_adc_volts(&mut self, channel: u32, volts: f64) {
        if let Some(slot) = self.analog.get_mut(channel as usize) {
            self.analog_invalid[channel as usize] = !volts.is_finite();
            if volts.is_finite() {
                *slot = Some(volts);
                if let Some(mcu) = &mut self.mcu {
                    mcu.set_adc_volts(channel as usize, volts);
                }
            }
        } else {
            self.fault = Some(BoardFault::Misconfigured("неверный канал АЦП".into()));
        }
    }

    fn pin_output(&self, pin: u32) -> Option<bool> {
        self.output_mcu()?.gpio_drive(pin)
    }

    fn gpio_mode(&self, pin: u32) -> Option<GpioMode> {
        if pin as usize >= self.digital.len() {
            return None;
        }
        Some(
            self.output_mcu()?
                .gpio_drive(pin)
                .map_or(GpioMode::Input, GpioMode::Output),
        )
    }

    fn pwm_duty(&self, port: u32) -> Option<f64> {
        self.pwm(port).map(|p| p.duty)
    }

    fn uart_take(&mut self) -> Vec<u8> {
        self.collect();
        crate::device::take_uart_events(&mut self.events)
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
        self.reset
    }

    fn exit_code(&self) -> Option<u32> {
        let mcu = self.mcu.as_ref()?;
        mcu.halt_cycle().filter(|&cycle| cycle <= self.cycle)?;
        self.powered.then(|| mcu.exit_code()).flatten()
    }

    fn watchdog_resets(&self) -> u32 {
        self.mcu.as_ref().map_or(0, Mcu::watchdog_resets)
    }

    fn write_config(&mut self, bytes: &[u8]) -> Result<(), BoardFault> {
        self.mcu
            .as_mut()
            .ok_or(BoardFault::NoImage)?
            .write_config(bytes)
            .map_err(|e| BoardFault::Misconfigured(e.to_string()))
    }

    fn run_cycles(&mut self, cycles: u64) -> Result<Advance, BoardFault> {
        let target = end_cycle(self.cycle, cycles)?;
        if !self.powered {
            if let Some(mcu) = &mut self.mcu {
                mcu.advance_unpowered(target);
            }
        } else {
            self.checked_mcu()?;
            let mcu = self.mcu.as_mut().ok_or(BoardFault::NoImage)?;
            if self.executing {
                // Старый интерпретатор заканчивает начатую инструкцию. Её оставшиеся
                // такты относятся к следующему интервалу хоста, не исполняются повторно.
                let remaining = target.saturating_sub(mcu.cycles());
                mcu.run_cycles(remaining);
            } else {
                mcu.run_with_paused_core(target);
            }
        }
        self.cycle = target;
        self.target = target as f64;
        self.collect_state();
        if let Some(mcu) = self.mcu.as_ref().filter(|_| self.powered)
            && mcu.halt_cycle().is_some_and(|cycle| cycle <= self.cycle)
        {
            match mcu.halt_reason() {
                Some(HaltReason::Breakpoint { pc }) => {
                    return Err(BoardFault::Breakpoint { pc: *pc });
                }
                Some(HaltReason::Fault(fault)) => {
                    return Err(BoardFault::Execution(fault.to_string()));
                }
                _ => {}
            }
        }
        Ok(Advance {
            cycle: self.cycle,
            bus_request_pending: false,
        })
    }

    fn apply(&mut self, event: InputEvent) -> Result<InputOutcome, BoardFault> {
        event_time(self.cycle, event.cycle)?;
        match event.input {
            BoardInput::Power { on } => {
                if self.brownout && on {
                    return Err(BoardFault::Unpowered);
                }
                if self.powered == on {
                    return Ok(InputOutcome::Unchanged);
                }
                self.set_power(on);
            }
            BoardInput::Brownout { asserted } => {
                if self.brownout == asserted {
                    return Ok(InputOutcome::Unchanged);
                }
                self.set_power(!asserted);
                self.brownout = asserted;
                if asserted {
                    self.reset = Some(ResetCause::Brownout);
                    self.event(BoardOutput::Reset(ResetCause::Brownout));
                }
            }
            BoardInput::Execution { enabled } => {
                if self.executing == enabled {
                    return Ok(InputOutcome::Unchanged);
                }
                self.executing = enabled;
                self.event(BoardOutput::Execution { enabled });
            }
            BoardInput::Gpio { pin, high } if (pin as usize) < self.digital.len() => {
                self.set_pin_input(pin, high);
            }
            BoardInput::Adc { channel, volts }
                if channel < self.capabilities().adc_channels && volts.is_finite() =>
            {
                self.set_adc_volts(channel, volts);
            }
            BoardInput::DisconnectAdc { channel } if channel < self.capabilities().adc_channels => {
                self.analog[channel as usize] = None;
                self.analog_invalid[channel as usize] = false;
                if let Some(mcu) = &mut self.mcu {
                    mcu.disconnect_adc(channel);
                }
            }
            BoardInput::Gpio { .. } | BoardInput::Adc { .. } | BoardInput::DisconnectAdc { .. } => {
                return Err(BoardFault::Misconfigured(
                    "неверный вход GPIO или АЦП".into(),
                ));
            }
            BoardInput::Uart { port: 0, bytes } => {
                if !self.powered {
                    return Err(BoardFault::Unpowered);
                }
                let accepted = self
                    .mcu
                    .as_mut()
                    .ok_or(BoardFault::NoImage)?
                    .uart_send(&bytes);
                return Ok(InputOutcome::Uart {
                    accepted,
                    dropped: bytes.len() - accepted,
                });
            }
            BoardInput::Radio { port: 0, packet } => {
                if !self.powered {
                    return Err(BoardFault::Unpowered);
                }
                let rssi = packet.rssi_dbm.ok_or_else(|| {
                    BoardFault::Misconfigured("у входного пакета нет RSSI".into())
                })?;
                if packet.bytes.is_empty()
                    || packet.bytes.len() > sim_mcu::periph::radio::MAX_PACKET
                {
                    return Err(BoardFault::Misconfigured("неверная длина пакета".into()));
                }
                let accepted = self
                    .mcu
                    .as_mut()
                    .ok_or(BoardFault::NoImage)?
                    .radio_deliver(&packet.bytes, rssi);
                if !accepted {
                    return Ok(InputOutcome::PacketDropped);
                }
            }
            _ => {
                return Err(BoardFault::Unsupported(
                    "порт отсутствует в виртуальном профиле".into(),
                ));
            }
        }
        Ok(InputOutcome::Applied)
    }

    fn pwm(&self, port: u32) -> Option<PwmOutput> {
        let mcu = self.output_mcu()?;
        let channel = port as usize;
        let hz = mcu.pwm_freq_hz(channel);
        (hz > 0 && mcu.pwm_enabled(channel))
            .then(|| PwmOutput::new(mcu.pwm_duty(channel), 1.0 / f64::from(hz)))
    }

    fn take_events(&mut self) -> Vec<OutputEvent> {
        self.collect();
        self.events.sort_by_key(|event| event.cycle);
        let ready = self
            .events
            .partition_point(|event| event.cycle <= self.cycle);
        self.events.drain(..ready).collect()
    }
}
