use std::sync::Arc;

use crate::esp_clock::QUARTERS_PER_SECOND;
use crate::ports::{OutputPort, PortSample, PortValue, end_cycle, event_time};
use crate::{
    Advance, BoardInput, BoardOutput, Capabilities, EspSleepStatus, GpioMode, InputEvent,
    InputOutcome, OutputEvent, PwmOutput, ResetCause, capabilities,
};
use crate::{
    BoardFault, BoardId, arm::Arm, avr::Avr, esp_rom, image, memory::Bus, peripheral::Hardware,
    riscv::Riscv,
};
use crate::{EspResetProfile, EspResetValueUse, EspVirtualRngUse, esp_rng::EspVirtualRng};

#[path = "esp_radio.rs"]
mod esp_radio;

/// Входы модели загрузочной периферии, которые не записаны в образе.
#[derive(Clone, Copy, Default)]
pub struct EspBootOptions {
    /// Не задана — обращение к SYSTIMER даёт явный отказ.
    pub systimer_phase: Option<crate::SystimerPhase>,
    pub flash_readback: Option<crate::esp_flash::EspFlashReadback>,
}

impl EspBootOptions {
    /// Подключает явно переданные состояния для чтений, которые иначе дают ошибку.
    pub fn with_reset_profile(self, profile: EspResetProfile) -> EspProfiledBootOptions {
        EspProfiledBootOptions {
            options: self,
            profile: Some(profile),
            rng_seed: None,
        }
    }

    /// Задаёт воспроизводимый источник чтений RNG_DATA_REG, сохраняя строгий сброс.
    pub fn with_virtual_rng_seed(self, seed: u64) -> EspProfiledBootOptions {
        EspProfiledBootOptions {
            options: self,
            profile: None,
            rng_seed: Some(seed),
        }
    }
}

/// Расширенные входы raw flash. Прежние литералы EspBootOptions совместимы.
pub struct EspProfiledBootOptions {
    options: EspBootOptions,
    profile: Option<EspResetProfile>,
    rng_seed: Option<u64>,
}

impl EspProfiledBootOptions {
    /// Включает воспроизводимый заменитель аппаратного RNG для запуска модели.
    pub fn with_virtual_rng_seed(mut self, seed: u64) -> Self {
        self.rng_seed = Some(seed);
        self
    }
}

/// Позиция в потоке шагов CPU после загрузки raw flash.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EspBootPosition {
    /// Индекс следующего шага от нуля. Включает шаги trap и WFI.
    pub step: u64,
    pub pc: u32,
}

/// Интерфейс исполнения прошивки. Успех run_for означает, что пройден
/// запрошенный интервал поддерживаемой модели.
pub trait BoardSim: Send {
    fn board(&self) -> BoardId;
    fn load(&mut self, image: &[u8]) -> Result<(), BoardFault>;
    /// Устанавливает raw flash и входы холодного старта ESP32-C3.
    /// `rom` — полный дамп ROM0/ROM1 в порядке IBUS. `efuse_readback` — слова
    /// EFUSE_RD_WR_DIS_REG..=EFUSE_RD_SYS_PART2_DATA7_REG. `straps` задаёт GPIO2,
    /// GPIO8, GPIO9, если соответствующий уровень не подан через GPIO-вход.
    /// Исполнение начинается с вектора ROM при первом такте. Успех загрузки
    /// означает проверку входов. Ошибки исполнения возвращаются из run_cycles.
    fn load_esp_flash(
        &mut self,
        flash: &[u8],
        partition_table_offset: u32,
        rom: &[u8],
        efuse_readback: &[u32],
        straps: [bool; 3],
    ) -> Result<(), BoardFault> {
        self.load_esp_flash_with_options(
            flash,
            partition_table_offset,
            rom,
            efuse_readback,
            straps,
            EspBootOptions::default(),
        )
    }
    fn load_esp_flash_with_options(
        &mut self,
        flash: &[u8],
        partition_table_offset: u32,
        rom: &[u8],
        efuse_readback: &[u32],
        straps: [bool; 3],
        options: EspBootOptions,
    ) -> Result<(), BoardFault> {
        self.load_esp_flash_with_profile(
            flash,
            partition_table_offset,
            rom,
            efuse_readback,
            straps,
            EspProfiledBootOptions {
                options,
                profile: None,
                rng_seed: None,
            },
        )
    }
    /// Загружает raw flash с явно выбранным профилем неизвестных состояний сброса.
    fn load_esp_flash_with_profile(
        &mut self,
        flash: &[u8],
        partition_table_offset: u32,
        rom: &[u8],
        efuse_readback: &[u32],
        straps: [bool; 3],
        options: EspProfiledBootOptions,
    ) -> Result<(), BoardFault> {
        let _ = (
            flash,
            partition_table_offset,
            rom,
            efuse_readback,
            straps,
            options,
        );
        Err(BoardFault::Unsupported(
            "профиль платы не поддерживает аппаратную загрузку ESP32-C3".into(),
        ))
    }
    /// Диагностика raw flash. После ошибки указывает шаг, на котором возник отказ.
    fn esp_boot_position(&self) -> Option<EspBootPosition> {
        None
    }
    /// Все обращения к профилю с момента установки текущего образа.
    /// Доступно и после ошибки. Эти данные описывают допущения модели.
    fn esp_reset_value_uses(&self) -> &[EspResetValueUse] {
        &[]
    }
    fn esp_virtual_rng_use(&self) -> Option<EspVirtualRngUse> {
        None
    }
    /// Состояние RTC для ESP32-C3 с raw flash. У прочих профилей и при снятом
    /// питании состояние отсутствует.
    fn esp_sleep_status(&self) -> Option<EspSleepStatus> {
        None
    }
    fn reset(&mut self);
    fn run_for(&mut self, seconds: f64) -> Result<(), BoardFault>;
    fn set_pin_input(&mut self, pin: u32, high: bool);
    fn set_adc_volts(&mut self, channel: u32, volts: f64);
    fn pin_output(&self, pin: u32) -> Option<bool>;
    /// Отсутствие значения означает, что периферия не установила модель контакта.
    fn gpio_mode(&self, pin: u32) -> Option<GpioMode> {
        self.pin_output(pin).map(GpioMode::Output)
    }
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
    /// Продвигает не более заданного числа тактов. Может остановиться на
    /// запросе шины или пакетного адаптера.
    /// В Light-sleep и Deep-sleep ESP32-C3 счётчик времени платы использует
    /// последний период CPU. Исполнение ядра при этом приостановлено.
    fn run_cycles(&mut self, cycles: u64) -> Result<Advance, BoardFault>;
    /// Время границы уже исполненного такта в четвертях наносекунды.
    /// None означает постоянную частоту из capabilities().
    fn cycle_time_quarter_ns(&self, cycle: u64) -> Result<Option<u128>, BoardFault> {
        let _ = cycle;
        Ok(None)
    }
    /// Исполняет целые такты, помещающиеся до абсолютной отметки платы,
    /// с пересчётом после смены частоты. Остановка совпадает с run_cycles.
    /// None означает постоянную частоту из capabilities().
    fn run_until_quarter_ns(&mut self, deadline: u128) -> Result<Option<Advance>, BoardFault> {
        let _ = deadline;
        Ok(None)
    }
    fn apply(&mut self, event: InputEvent) -> Result<InputOutcome, BoardFault>;
    fn pwm(&self, port: u32) -> Option<PwmOutput>;
    fn take_events(&mut self) -> Vec<OutputEvent>;
    /// Гостевой пакетный адаптер открыл вход. У прежних профилей состояние
    /// аппаратного MAC через этот метод не определяется.
    fn esp_radio_listening(&self, port: u32) -> bool {
        let _ = port;
        false
    }
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
        brownout: false,
        executing: true,
        image: None,
        cycle: 0,
        reset_cause: None,
        watchdog_resets: 0,
        next_bus_id: 0,
        events: Vec::new(),
        clock_timeline: None,
        reset_value_history: Vec::new(),
        rng_history: None,
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
enum Firmware {
    Legacy(Arc<[u8]>),
    EspFlash(Box<EspFirmware>),
}

#[derive(Clone)]
struct EspFirmware {
    image: Arc<esp_rom::BootImage>,
    flash: crate::esp_flash::EspFlash,
    systimer_phase: Option<crate::SystimerPhase>,
    wdt_delay_sel: u8,
    reset_profile: Option<EspResetProfile>,
    rng_seed: Option<u64>,
}

#[derive(Clone)]
struct RomBoot {
    image: Arc<esp_rom::BootImage>,
    in_rom: bool,
    step: u64,
    fault_pc: Option<u32>,
}

#[derive(Clone, Copy)]
struct ClockSegment {
    cycle: u64,
    time: u128,
    period: u32,
}

#[derive(Clone)]
struct ClockTimeline(Arc<Vec<ClockSegment>>);

// Шкала для совместимых ELF и продвижения счётчика при выключенной плате.
fn fixed_esp_period() -> u32 {
    QUARTERS_PER_SECOND / Hardware::frequency(BoardId::Esp32C3) as u32
}

impl ClockTimeline {
    fn new() -> Self {
        Self(Arc::new(vec![ClockSegment {
            cycle: 0,
            time: 0,
            period: fixed_esp_period(),
        }]))
    }

    fn at(&self, cycle: u64) -> u128 {
        let segment = self.0[self.0.partition_point(|point| point.cycle <= cycle) - 1];
        segment.time + u128::from(cycle - segment.cycle) * u128::from(segment.period)
    }

    fn set_period(&mut self, cycle: u64, period: u32) {
        let previous = self.0.last().expect("шкала начинается с такта ноль");
        if previous.period == period {
            return;
        }
        let point = ClockSegment {
            cycle,
            time: self.at(cycle),
            period,
        };
        let points = Arc::make_mut(&mut self.0);
        if points
            .last()
            .is_some_and(|previous| previous.cycle == cycle)
        {
            *points.last_mut().unwrap() = point;
        } else {
            points.push(point);
        }
    }
}

fn cycles_before(deadline: u128, now: u128, period: u32) -> u64 {
    u64::try_from(deadline.saturating_sub(now) / u128::from(period)).unwrap_or(u64::MAX)
}

#[derive(Clone)]
struct Machine {
    bus: Bus,
    cpu: Cpu,
    entry: u32,
    rom_boot: Option<RomBoot>,
    remaining: u64,
    fractional: f64,
    radio: esp_radio::Driver,
}

impl Machine {
    fn restart(
        &mut self,
        watchdog: bool,
        chip_reset_pins: Option<&[Option<bool>]>,
    ) -> Result<(), BoardFault> {
        self.radio.reset(self.bus.hw.cycles);
        self.bus.hw.restart(watchdog);
        if self.bus.id == BoardId::Rp2040 {
            self.bus.hw.vector_base = self.entry;
        }
        if let Some(boot) = &mut self.rom_boot {
            // TRM §6.1: при Chip Reset SRAM теряет содержимое и фиксируются
            // новые strap. Другие уровни сброса сохраняют внутреннюю память.
            if let Some(pins) = chip_reset_pins {
                self.bus.esp_memory = Some(boot.image.cold_memory(pins));
            }
            boot.in_rom = true;
            boot.step = 0;
            boot.fault_pc = None;
        }
        self.cpu = Cpu::boot(&mut self.bus, self.entry)?;
        Ok(())
    }

    fn step(&mut self) -> Result<u64, BoardFault> {
        if let (Some(boot), Cpu::Riscv(cpu), Some(memory)) =
            (&mut self.rom_boot, &self.cpu, &mut self.bus.esp_memory)
        {
            boot.fault_pc = Some(cpu.pc);
            self.bus.hw.esp_boot.as_mut().expect("установлена flash").pc = cpu.pc;
            if boot.in_rom && boot.step == 0 {
                boot.image.check_mode(memory)?;
                self.bus.hw.enter_esp_flash_boot()?;
            }
        }
        match self.cpu.step(&mut self.bus) {
            Err(BoardFault::UnknownInstruction { pc, raw, .. })
                if raw == crate::EspRadioAbi::INSTRUCTION
                    && self.rom_boot.as_ref().is_none_or(|boot| !boot.in_rom) =>
            {
                let Cpu::Riscv(cpu) = &mut self.cpu else {
                    unreachable!("код custom-0 относится к RISC-V");
                };
                let next = pc
                    .checked_add(crate::EspRadioAbi::CALL_BYTES)
                    .ok_or_else(|| {
                        BoardFault::Execution("PC вызова пакетного адаптера вне диапазона".into())
                    })?;
                self.radio.call(&mut self.bus, pc)?;
                cpu.pc = next;
                // Та же оценка, что у Riscv::step: один такт на инструкцию,
                // без утверждения о времени исполнения SDK на кремнии.
                Ok(1)
            }
            result => result,
        }
    }

    fn restart_after_sleep(&mut self, retain_fast: bool) -> Result<(), BoardFault> {
        let boot = self
            .rom_boot
            .as_mut()
            .expect("Deep-sleep доступен с raw flash");
        let old = self.bus.esp_memory.as_mut().expect("установлена память C3");
        // TRM §§5.4, 6.1 и рег. 5.8. Deep-sleep сбрасывает цифровой домен, но не
        // перечитывает внешние strap. Содержимое цифровой SRAM теряется.
        let straps = old.read(self.entry, 0x6000_4038, size_of::<u32>() as u8)?;
        let pins = std::array::from_fn::<_, 22, _>(|pin| match pin {
            2 => Some(straps & 1 != 0),
            8 => Some(straps & (1 << 2) != 0),
            9 => Some(straps & (1 << 3) != 0),
            _ => None,
        });
        let mut memory = boot.image.cold_memory(&pins);
        if retain_fast {
            crate::peripheral::esp_sleep::retain_rtc_fast(old, &mut memory)?;
        }
        self.bus.hw.finish_deep_wake();
        self.radio.reset(self.bus.hw.cycles);
        self.bus.esp_memory = Some(memory);
        boot.in_rom = true;
        boot.step = 0;
        boot.fault_pc = None;
        self.cpu = Cpu::boot(&mut self.bus, self.entry)?;
        Ok(())
    }

    fn restart_if_requested(&mut self) -> Result<(), BoardFault> {
        if self.bus.hw.reset_requested {
            let watchdog = self.bus.hw.watchdog_reset();
            self.restart(watchdog, None)?;
            if !watchdog {
                let hw = &mut self.bus.hw;
                hw.reset_events.push(OutputEvent {
                    cycle: hw.cycles,
                    output: BoardOutput::Reset(ResetCause::Software),
                });
            }
            self.remaining = 0;
        }
        Ok(())
    }

    fn run(
        &mut self,
        cycles: u64,
        executing: bool,
        mut timeline: Option<&mut ClockTimeline>,
        deadline: Option<u128>,
    ) -> Result<(), BoardFault> {
        let mut todo = cycles;
        while todo != 0 && !self.has_request() {
            let rates = self
                .bus
                .hw
                .esp_boot
                .as_ref()
                .map(|boot| boot.system.clocks());
            let period = rates.map_or_else(fixed_esp_period, |clock| clock.cpu_cycle_quarters());
            let mut available = todo;
            if let Some(timeline) = timeline.as_deref_mut() {
                let cycle = self.bus.hw.cycles;
                if let Some(deadline) = deadline {
                    available = available.min(cycles_before(deadline, timeline.at(cycle), period));
                    if available == 0 {
                        break;
                    }
                }
                timeline.set_period(cycle, period);
            }
            if self.remaining == 0 && self.bus.hw.sleeping_mode().is_some() {
                // В спящем режиме такты интерфейса продолжают шкалу хоста
                // с последним периодом CPU. Инструкции и цифровые таймеры стоят.
                let (spend, deep_wake) = self.bus.hw.advance_sleep(available, period)?;
                todo -= spend;
                if let Some(retain_fast) = deep_wake {
                    self.restart_after_sleep(retain_fast)?;
                }
                self.restart_if_requested()?;
                continue;
            }
            let spend;
            if !executing {
                spend = 1;
            } else {
                if self.remaining == 0 {
                    self.remaining = self.step()?;
                }
                spend = self.remaining.min(available);
                self.remaining -= spend;
            }
            if let Some(rates) = rates {
                let system = &self.bus.hw.esp_boot.as_ref().unwrap().system;
                let stalled = !executing
                    || matches!(&self.cpu, Cpu::Riscv(cpu) if cpu.clock_stalled(system.wait_gate_cycles()));
                self.bus.hw.advance_esp_boot(spend, rates, stalled)?;
            }
            self.bus.hw.tick(spend);
            if self.remaining == 0 {
                self.bus.hw.sleep_commit_entry();
            }
            todo -= spend;
            if executing {
                if self.remaining == 0
                    && let (Some(boot), Cpu::Riscv(cpu), Some(memory)) =
                        (&mut self.rom_boot, &mut self.cpu, &mut self.bus.esp_memory)
                {
                    if boot.in_rom {
                        boot.image
                            .check_transfer(&mut boot.in_rom, cpu.pc, memory)?;
                        if !boot.in_rom {
                            cpu.finish_rom_handoff();
                        }
                    }
                    boot.step += 1;
                    boot.fault_pc = None;
                }
            }
            if let Some(fault) = self.bus.hw.bus_fault() {
                return Err(fault.clone());
            }
            self.restart_if_requested()?;
        }
        Ok(())
    }

    fn has_request(&self) -> bool {
        self.bus.hw.has_bus_request() || self.radio.request
    }

    fn take_requests(&mut self, events: &mut Vec<OutputEvent>) {
        events.extend(self.bus.hw.take_bus_requests());
        self.radio.request = false;
        events.append(&mut self.radio.events);
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
    brownout: bool,
    executing: bool,
    image: Option<Firmware>,
    cycle: u64,
    reset_cause: Option<ResetCause>,
    watchdog_resets: u32,
    next_bus_id: u64,
    events: Vec<OutputEvent>,
    clock_timeline: Option<ClockTimeline>,
    reset_value_history: Vec<EspResetValueUse>,
    rng_history: Option<EspVirtualRng>,
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
            machine.radio.reset(self.cycle);
            machine.take_requests(&mut self.events);
            let hw = &mut machine.bus.hw;
            hw.cancel_bus();
            self.events.extend(hw.take_uart_events());
            self.events.extend(hw.take_bus_requests());
            self.events.extend(hw.take_bus_completed());
            self.events.extend(hw.take_reset_events());
            self.next_bus_id = hw.next_bus_id(None);
            if let Some(boot) = hw.esp_boot.as_mut() {
                self.reset_value_history = boot.reset_values.take_usage();
                self.rng_history = boot.virtual_rng.take();
            }
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
            self.start(&image)?;
            self.emit(BoardOutput::Reset(ResetCause::PowerOn));
        }
        Ok(())
    }

    fn replace_firmware(
        &mut self,
        prepare: impl FnOnce() -> Result<Firmware, BoardFault>,
    ) -> Result<(), BoardFault> {
        // Не оставляем прежнюю прошивку доступной для запуска после неудачной записи.
        self.disconnect();
        self.reset_value_history.clear();
        self.rng_history = None;
        self.image = None;
        self.reset_cause = None;
        if !self.powered {
            return Err(BoardFault::Unpowered);
        }
        let image = prepare().inspect_err(|error| self.fault = Some(error.clone()))?;
        self.start(&image)?;
        self.image = Some(image);
        Ok(())
    }

    fn start(&mut self, firmware: &Firmware) -> Result<(), BoardFault> {
        self.reset_cause = None;
        let loaded: Result<Machine, BoardFault> = (|| {
            let (mut bus, entry, rom_boot) = match firmware {
                Firmware::Legacy(data) => {
                    let image = self.parse_legacy(data)?;
                    let mut bus = Bus::new(self.id, &image)?;
                    let entry = Self::legacy_entry(&mut bus, &image)?;
                    (bus, entry, None)
                }
                Firmware::EspFlash(firmware) => {
                    let mut bus = Bus::new_esp_flash(firmware.image.cold_memory(&self.digital));
                    bus.hw.install_esp_boot(
                        firmware.flash.clone(),
                        firmware.systimer_phase,
                        firmware.wdt_delay_sel,
                        firmware.reset_profile.clone(),
                    )?;
                    bus.hw.esp_boot.as_mut().unwrap().virtual_rng =
                        firmware.rng_seed.map(EspVirtualRng::new);
                    let boot = RomBoot {
                        image: Arc::clone(&firmware.image),
                        in_rom: true,
                        step: 0,
                        fault_pc: None,
                    };
                    (bus, esp_rom::RESET_VECTOR, Some(boot))
                }
            };
            bus.hw.cycles = self.cycle;
            bus.hw.next_bus_id(Some(self.next_bus_id));
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
                rom_boot,
                remaining: 0,
                fractional: 0.0,
                radio: esp_radio::Driver::default(),
            })
        })();
        match loaded {
            Ok(mut machine) => {
                if machine.rom_boot.is_some() {
                    self.clock_timeline.get_or_insert_with(ClockTimeline::new);
                    if let Some(boot) = machine.bus.hw.esp_boot.as_mut() {
                        boot.reset_values
                            .restore_usage(std::mem::take(&mut self.reset_value_history));
                        if self.rng_history.is_some() {
                            boot.virtual_rng = self.rng_history.take();
                        }
                    }
                }
                self.machine = Some(machine);
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

    fn parse_legacy(&self, data: &[u8]) -> Result<image::Image, BoardFault> {
        let arch = match self.id {
            BoardId::Virtual => unreachable!("виртуальный профиль имеет собственный загрузчик"),
            BoardId::Rp2040 | BoardId::Stm32f103c8 => image::Machine::Arm,
            BoardId::Esp32C3 => image::Machine::RiscV,
            BoardId::Atmega328p => image::Machine::Avr,
        };
        if arch == image::Machine::Avr && !image::is_elf(data) {
            image::parse_ihex(data)
        } else {
            image::parse_elf(data, arch)
        }
    }

    fn legacy_entry(bus: &mut Bus, image: &image::Image) -> Result<u32, BoardFault> {
        Ok(match bus.id {
            BoardId::Rp2040 => {
                // Pico SDK: 256 байт boot2, затем таблица векторов. Поддержан и ELF
                // стенда с таблицей векторов непосредственно в начале XIP.
                let at = 0x1000_0000;
                let mut plausible = [at, at + 0x100]
                    .into_iter()
                    .filter(|&base| Arm::boot(bus, base).is_ok());
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
        })
    }

    fn advance(&mut self, count: u64, deadline: Option<u128>) -> Result<Advance, BoardFault> {
        let target = end_cycle(self.cycle, count)?;
        if !self.powered {
            if let Some(timeline) = &mut self.clock_timeline {
                timeline.set_period(self.cycle, fixed_esp_period());
            }
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
        let result = machine.run(
            count,
            self.executing,
            self.clock_timeline.as_mut(),
            deadline,
        );
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
        if let Err(fault) = result {
            self.fault = Some(fault.clone());
            self.emit(BoardOutput::Fault(fault.clone()));
            return Err(fault);
        }
        Ok(Advance {
            cycle: self.cycle,
            bus_request_pending: machine.has_request(),
        })
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
        self.replace_firmware(|| Ok(Firmware::Legacy(data.into())))
    }

    fn load_esp_flash_with_profile(
        &mut self,
        flash: &[u8],
        partition_table_offset: u32,
        rom: &[u8],
        efuse_readback: &[u32],
        straps: [bool; 3],
        options: EspProfiledBootOptions,
    ) -> Result<(), BoardFault> {
        if self.id != BoardId::Esp32C3 {
            return Err(BoardFault::Unsupported(
                "профиль платы не поддерживает аппаратную загрузку ESP32-C3".into(),
            ));
        }
        self.replace_firmware(|| {
            let image = esp_rom::BootImage::new(
                flash,
                partition_table_offset,
                rom,
                efuse_readback,
                straps,
            )?;
            let parsed = crate::esp_image::parse_flash(flash, partition_table_offset)?;
            let mut flash = crate::esp_flash::EspFlash::from_image(&parsed)?;
            if let Some(readback) = options.options.flash_readback {
                flash.set_readback(readback)?;
            }
            // ESP-IDF soc/esp32c3/register/soc/efuse_reg.h:
            // EFUSE_RD_REPEAT_DATA1 (0x34), EFUSE_WDT_DELAY_SEL[17:16].
            // BootImage уже проверил размер массива от EFUSE_RD_WR_DIS (0x2c).
            let word = efuse_readback[(0x34 - 0x2c) / size_of::<u32>()];
            Ok(Firmware::EspFlash(Box::new(EspFirmware {
                image: Arc::new(image),
                flash,
                systimer_phase: options.options.systimer_phase,
                wdt_delay_sel: ((word >> 16) & 0b11) as u8,
                reset_profile: options.profile,
                rng_seed: options.rng_seed,
            })))
        })
    }

    fn esp_boot_position(&self) -> Option<EspBootPosition> {
        let machine = self.machine.as_ref()?;
        let boot = machine.rom_boot.as_ref()?;
        match &machine.cpu {
            Cpu::Riscv(cpu) => Some(EspBootPosition {
                step: boot.step,
                pc: boot.fault_pc.unwrap_or(cpu.pc),
            }),
            _ => None,
        }
    }

    fn esp_reset_value_uses(&self) -> &[EspResetValueUse] {
        self.machine
            .as_ref()
            .and_then(|machine| machine.bus.hw.esp_boot.as_ref())
            .map_or(self.reset_value_history.as_slice(), |boot| {
                boot.reset_values.usage()
            })
    }

    fn esp_virtual_rng_use(&self) -> Option<EspVirtualRngUse> {
        self.machine
            .as_ref()
            .and_then(|machine| machine.bus.hw.esp_boot.as_ref())
            .and_then(|boot| boot.virtual_rng.as_ref())
            .or(self.rng_history.as_ref())
            .map(EspVirtualRng::usage)
    }

    fn esp_sleep_status(&self) -> Option<EspSleepStatus> {
        if !self.powered || self.fault.is_some() {
            return None;
        }
        self.machine.as_ref()?.bus.hw.esp_sleep_status()
    }

    fn reset(&mut self) {
        if self.powered
            && let Some(machine) = self.machine.as_mut()
        {
            self.fault = machine.restart(false, Some(&self.digital)).err();
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
            if let Some(timeline) = &self.clock_timeline {
                let machine = self.machine.as_mut().ok_or(BoardFault::NoImage)?;
                let quarters = seconds * f64::from(QUARTERS_PER_SECOND) + machine.fractional;
                if !quarters.is_finite() || quarters >= (1u64 << 53) as f64 {
                    return Err(BoardFault::Misconfigured(
                        "интервал вне точного диапазона четвертей наносекунды".into(),
                    ));
                }
                let deadline = timeline.at(self.cycle) + quarters as u128;
                while self
                    .run_until_quarter_ns(deadline)?
                    .is_some_and(|p| p.bus_request_pending)
                {
                    self.machine
                        .as_mut()
                        .expect("плата запущена")
                        .take_requests(&mut self.events);
                }
                let now = self.clock_timeline.as_ref().unwrap().at(self.cycle);
                self.machine.as_mut().unwrap().fractional =
                    (deadline - now) as f64 + quarters.fract();
                return Ok(());
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
                    machine.take_requests(&mut self.events);
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

    fn gpio_mode(&self, pin: u32) -> Option<GpioMode> {
        if !self.powered || self.fault.is_some() {
            return None;
        }
        self.machine.as_ref()?.bus.hw.gpio_mode(pin)
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
        self.advance(count, None)
    }

    fn cycle_time_quarter_ns(&self, cycle: u64) -> Result<Option<u128>, BoardFault> {
        let Some(timeline) = &self.clock_timeline else {
            return Ok(None);
        };
        if cycle > self.cycle {
            return Err(BoardFault::Misconfigured(
                "такт платы ещё не исполнен".into(),
            ));
        }
        Ok(Some(timeline.at(cycle)))
    }

    fn run_until_quarter_ns(&mut self, deadline: u128) -> Result<Option<Advance>, BoardFault> {
        let Some(timeline) = &self.clock_timeline else {
            return Ok(None);
        };
        let count = if self.powered {
            // Проверка точной границы end_cycle проводится до изменения CPU.
            // TRM §6.2.4.1: кратчайший период CPU соответствует 160 МГц.
            const MIN_PERIOD: u32 = QUARTERS_PER_SECOND / 160_000_000;
            cycles_before(deadline, timeline.at(self.cycle), MIN_PERIOD)
        } else {
            cycles_before(deadline, timeline.at(self.cycle), fixed_esp_period())
        };
        self.advance(count, self.powered.then_some(deadline))
            .map(Some)
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
                self.set_power(on)?;
            }
            BoardInput::Brownout { asserted } => {
                if self.brownout == asserted {
                    return Ok(InputOutcome::Unchanged);
                }
                self.set_power(!asserted)?;
                self.brownout = asserted;
                if asserted {
                    self.reset_cause = Some(ResetCause::Brownout);
                    self.emit(BoardOutput::Reset(ResetCause::Brownout));
                }
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
                return match input {
                    BoardInput::Radio { port, packet } if self.id == BoardId::Esp32C3 => {
                        if machine.bus.hw.sleeping_mode().is_some() {
                            Ok(InputOutcome::PacketDropped)
                        } else {
                            machine.radio.receive(port, packet)
                        }
                    }
                    BoardInput::RadioReplyId { port, id } if self.id == BoardId::Esp32C3 => {
                        machine.radio.reply_id(port, id)
                    }
                    BoardInput::RadioResult {
                        port,
                        sent_cycle,
                        status,
                    } if self.id == BoardId::Esp32C3 => {
                        let status = if machine.bus.hw.sleeping_mode().is_some() {
                            crate::RadioStatus::NotAnalyzed("получение подтверждения во время сна")
                        } else {
                            status
                        };
                        machine.radio.result(self.cycle, port, sent_cycle, status)
                    }
                    input => machine.bus.hw.apply_port(input),
                };
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
            machine.take_requests(&mut self.events);
            let hw = &mut machine.bus.hw;
            self.events.extend(hw.take_uart_events());
            self.events.extend(hw.take_bus_completed());
            self.events.extend(hw.take_reset_events());
        }
        self.events.sort_by_key(|event| event.cycle);
        std::mem::take(&mut self.events)
    }

    fn esp_radio_listening(&self, port: u32) -> bool {
        self.powered
            && self.executing
            && self.fault.is_none()
            && self.machine.as_ref().is_some_and(|machine| {
                machine.bus.hw.sleeping_mode().is_none() && machine.radio.listening(port)
            })
    }
}
