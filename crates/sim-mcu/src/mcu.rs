//! Машина целиком (ядро + шина + периферия) и программный интерфейс хоста.
//!
//! Хост (физический цикл приложения) задаёт входы (напряжения АЦП, уровни GPIO,
//! пакеты радио, байты UART), вызывает [`Mcu::run_for`] на шаг своего времени и
//! читает выходы (ШИМ, GPIO, вывод UART, пакеты радио). Время МК идёт только
//! внутри `run_for`, поэтому ко-симуляция детерминирована.
//!
//! `wfi` проматывает время до ближайшего события (таймер, сторож, конец шага) —
//! ядро, которое в основном спит, эмулируется в десятки раз быстрее реального.

use std::collections::BTreeMap;
use std::fmt;

use crate::bus::{Bus, CONFIG_SIZE, CPU_HZ, FLASH_BASE, FLASH_SIZE, RAM_BASE, RAM_SIZE};
use crate::cpu::{self, Clock, Cpu, Exit, cause};
use crate::elf::{self, ElfError};
use crate::periph::Request;
pub use crate::periph::sysctrl::ResetReason;
use crate::periph::{adc, gpio, pwm};

/// Ошибки интерфейса хоста (не сбои прошивки — те в [`Fault`]).
#[derive(Clone, Debug, PartialEq)]
pub enum McuError {
    Elf(ElfError),
    /// Загружаемый сегмент не помещается ни во флеш, ни в ОЗУ.
    SegmentOutOfRange {
        paddr: u32,
        len: u32,
    },
    /// Точка входа вне исполняемой памяти.
    EntryOutOfRange(u32),
    /// Блок конфигурации больше 4096 байт.
    ConfigTooLarge(usize),
    /// Длительность шага — отрицательная, NaN или бесконечность.
    BadDuration(f64),
}

impl fmt::Display for McuError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            McuError::Elf(e) => write!(f, "{e}"),
            McuError::SegmentOutOfRange { paddr, len } => write!(
                f,
                "сегмент 0x{paddr:08x}+{len} не помещается ни во флеш, ни в ОЗУ"
            ),
            McuError::EntryOutOfRange(e) => write!(f, "точка входа 0x{e:08x} вне флеша и ОЗУ"),
            McuError::ConfigTooLarge(n) => {
                write!(f, "блок конфигурации {n} байт > {CONFIG_SIZE}")
            }
            McuError::BadDuration(s) => write!(f, "недопустимая длительность шага {s} с"),
        }
    }
}

impl std::error::Error for McuError {}

impl From<ElfError> for McuError {
    fn from(e: ElfError) -> Self {
        McuError::Elf(e)
    }
}

/// Вид сбоя.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FaultKind {
    InstructionAccessFault,
    IllegalInstruction,
    LoadMisaligned,
    LoadAccessFault,
    StoreMisaligned,
    StoreAccessFault,
    /// Другая причина исключения (код `mcause`).
    Other(u32),
}

impl FaultKind {
    pub fn from_cause(c: u32) -> FaultKind {
        match c {
            cause::INSTR_ACCESS_FAULT => FaultKind::InstructionAccessFault,
            cause::ILLEGAL_INSTRUCTION => FaultKind::IllegalInstruction,
            cause::LOAD_MISALIGNED => FaultKind::LoadMisaligned,
            cause::LOAD_ACCESS_FAULT => FaultKind::LoadAccessFault,
            cause::STORE_MISALIGNED => FaultKind::StoreMisaligned,
            cause::STORE_ACCESS_FAULT => FaultKind::StoreAccessFault,
            c => FaultKind::Other(c),
        }
    }

    pub fn describe(&self) -> String {
        match self {
            FaultKind::InstructionAccessFault => "ошибка доступа при выборке инструкции".into(),
            FaultKind::IllegalInstruction => "недопустимая инструкция".into(),
            FaultKind::LoadMisaligned => "невыровненная загрузка".into(),
            FaultKind::LoadAccessFault => "ошибка доступа при загрузке".into(),
            FaultKind::StoreMisaligned => "невыровненная запись".into(),
            FaultKind::StoreAccessFault => "ошибка доступа при записи".into(),
            FaultKind::Other(c) => format!("исключение mcause={c}"),
        }
    }
}

/// Кто обнаружил сбой.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FaultSource {
    /// Эмулятор: исключение внутри обработчика исключения («двойной сбой») или
    /// одно и то же исключение по одному адресу много раз подряд (зацикливание
    /// ловушек). `first_*` — первое исключение цепочки.
    TrapLoop { first_cause: u32, first_pc: u32 },
    /// Прошивка: её обработчик исключений сообщил через SYSCTRL.FAULT_*.
    Firmware,
}

/// Сбой, остановивший ядро.
#[derive(Clone, Debug, PartialEq)]
pub struct Fault {
    pub kind: FaultKind,
    /// `mcause`.
    pub cause: u32,
    /// Адрес инструкции, вызвавшей сбой.
    pub pc: u32,
    /// `mtval`: адрес доступа или биты инструкции.
    pub tval: u32,
    pub source: FaultSource,
    /// Время модели в момент сбоя, с.
    pub time_s: f64,
}

impl fmt::Display for Fault {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{} (mcause={}) по адресу pc=0x{:08x}, mtval=0x{:08x}, t={:.6} с",
            self.kind.describe(),
            self.cause,
            self.pc,
            self.tval,
            self.time_s
        )?;
        match self.source {
            FaultSource::TrapLoop {
                first_cause,
                first_pc,
            } => write!(
                f,
                "; зацикливание ловушек, первое исключение mcause={first_cause} pc=0x{first_pc:08x}"
            ),
            FaultSource::Firmware => write!(f, "; сообщено обработчиком прошивки"),
        }
    }
}

/// Почему ядро остановлено.
#[derive(Clone, Debug, PartialEq)]
pub enum HaltReason {
    /// Прошивка записала код в SYSCTRL.HALT (конец теста). Ядро и периферия
    /// заморожены (сторож тоже); время модели идёт, чтобы оставаться в такте с хостом.
    Exit(u32),
    /// `ebreak` — остановка в отладчик. Заморожено, как при `Exit`; pc указывает на ebreak.
    Breakpoint { pc: u32 },
    /// Сбой. Ядро стоит, но периферия держит последние значения, а включённый
    /// сторож продолжает отсчёт и сбросит микросхему — как в железе.
    Fault(Fault),
}

/// Итог шага `run_for`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RunReport {
    /// Тактов прошло (включая сон).
    pub cycles: u64,
    /// Инструкций выполнено.
    pub instructions: u64,
    /// Ядро остановлено (см. [`Mcu::halt_reason`]).
    pub halted: bool,
}

/// Виртуальный микроконтроллер VMCU-C3.
#[derive(Clone, Debug)]
pub struct Mcu {
    cpu: Cpu,
    bus: Bus,
    clk: Clock,
    entry: u32,
    /// Содержимое ОЗУ из ELF (если сегменты грузятся прямо в ОЗУ) — для сброса по питанию.
    ram_init: Vec<(usize, Vec<u8>)>,
    symbols: BTreeMap<String, u32>,
    halt: Option<HaltReason>,
    halt_cycle: Option<u64>,
    fault: Option<Fault>,
    /// Накопленная цель по времени в тактах (дробная — чтобы не копить ошибку округления).
    target_cycles: f64,
    // Учёт загрузки ЦП.
    active_total: u64,
    sleep_total: u64,
    win_active: u64,
    win_total: u64,
    window_cycles: u64,
    last_load: Option<f64>,
    host_resets: Vec<(u64, ResetReason)>,
    host_faults: Vec<(u64, Fault)>,
}

impl Mcu {
    /// Загружает ELF32 RISC-V: сегменты `PT_LOAD` кладутся по физическим адресам
    /// (как это делает прошивальщик: `.data` — во флеш, стартовый код копирует её
    /// в ОЗУ). Сразу выполняется сброс по питанию.
    pub fn from_elf(bytes: &[u8]) -> Result<Mcu, McuError> {
        let img = elf::parse(bytes)?;
        // ELF gABI, p_flags: PF_X = 1. Пустую флеш или ОЗУ нельзя принять
        // за начало программы, даже когда адрес попадает в карту памяти.
        if img.entry & 1 != 0
            || !img.segments.iter().any(|segment| {
                segment.flags & 1 != 0
                    && img.entry.checked_sub(segment.paddr).is_some_and(|offset| {
                        segment.data.len().saturating_sub(offset as usize) >= 2
                    })
            })
        {
            return Err(McuError::EntryOutOfRange(img.entry));
        }
        let mut bus = Bus::new();
        let mut ram_init = Vec::new();
        let mut code_end = 0usize;
        for s in &img.segments {
            if s.data.is_empty() {
                continue; // .bss и стек — обнуляет стартовый код
            }
            let len = s.data.len();
            let f = s.paddr.wrapping_sub(FLASH_BASE) as usize;
            let r = s.paddr.wrapping_sub(RAM_BASE) as usize;
            if f < FLASH_SIZE && f + len <= FLASH_SIZE {
                bus.flash[f..f + len].copy_from_slice(&s.data);
                code_end = code_end.max(f + len);
            } else if r < RAM_SIZE && r + len <= RAM_SIZE {
                ram_init.push((r, s.data.clone()));
            } else {
                return Err(McuError::SegmentOutOfRange {
                    paddr: s.paddr,
                    len: len as u32,
                });
            }
        }
        let symbols = img
            .symbols
            .iter()
            .filter(|s| s.kind <= 2)
            .map(|s| (s.name.clone(), s.value))
            .collect();
        Mcu::build(bus, code_end, img.entry, ram_init, symbols)
    }

    /// Образ без ELF: `flash` кладётся с начала флеша (0x4200_0000), исполнение —
    /// с `entry`. Удобно для покомандных тестов.
    pub fn from_image(flash: &[u8], entry: u32) -> Result<Mcu, McuError> {
        if flash.len() > FLASH_SIZE {
            return Err(McuError::SegmentOutOfRange {
                paddr: FLASH_BASE,
                len: flash.len() as u32,
            });
        }
        let mut bus = Bus::new();
        bus.flash[..flash.len()].copy_from_slice(flash);
        Mcu::build(bus, flash.len(), entry, Vec::new(), BTreeMap::new())
    }

    fn build(
        bus: Bus,
        code_end: usize,
        entry: u32,
        ram_init: Vec<(usize, Vec<u8>)>,
        symbols: BTreeMap<String, u32>,
    ) -> Result<Mcu, McuError> {
        if bus.fetch16(entry).is_none() {
            return Err(McuError::EntryOutOfRange(entry));
        }
        let cpu = Cpu::new(&bus, code_end);
        let mut m = Mcu {
            cpu,
            bus,
            clk: Clock::default(),
            entry,
            ram_init,
            symbols,
            halt: None,
            halt_cycle: None,
            fault: None,
            target_cycles: 0.0,
            active_total: 0,
            sleep_total: 0,
            win_active: 0,
            win_total: 0,
            window_cycles: CPU_HZ / 100,
            last_load: None,
            host_resets: Vec::new(),
            host_faults: Vec::new(),
        };
        m.reset();
        Ok(m)
    }

    /// Записывает блок конфигурации (0x5000_0000, до 4096 байт; остаток — 0xFF).
    /// Блок переживает любые сбросы; прошивка читает его при старте — поэтому
    /// после записи обычно вызывают [`Mcu::reset`].
    pub fn write_config(&mut self, bytes: &[u8]) -> Result<(), McuError> {
        if bytes.len() > CONFIG_SIZE {
            return Err(McuError::ConfigTooLarge(bytes.len()));
        }
        self.bus.config.fill(0xff);
        self.bus.config[..bytes.len()].copy_from_slice(bytes);
        Ok(())
    }

    /// Сброс по питанию: ОЗУ обнуляется (и заново заполняется сегментами ELF,
    /// лежащими в ОЗУ), периферия и ядро — в исходное, счётчики сбросов — в 0.
    /// Блок конфигурации, входы от хоста и время модели сохраняются.
    pub fn reset(&mut self) {
        self.bus.ram.fill(0);
        for (off, data) in &self.ram_init {
            self.bus.ram[*off..*off + data.len()].copy_from_slice(data);
        }
        self.bus.p.sys = Default::default();
        self.bus.p.reset_chip();
        self.cpu.reset(self.entry);
        self.halt = None;
        self.halt_cycle = None;
        self.fault = None;
        self.target_cycles = self.target_cycles.max(self.clk.cycle as f64);
    }

    /// Сброс микросхемы сторожем или программно: ОЗУ не трогается.
    fn chip_reset(&mut self, reason: ResetReason) {
        if self.bus.p.host_timestamps {
            self.host_resets.push((self.clk.cycle, reason));
        }
        let sys = &mut self.bus.p.sys;
        if reason == ResetReason::Watchdog {
            sys.wdt_resets = sys.wdt_resets.wrapping_add(1);
        }
        sys.reset_reason = reason;
        sys.boot_count = sys.boot_count.wrapping_add(1);
        self.bus.p.reset_chip();
        self.cpu.reset(self.entry);
        self.halt = None;
        self.halt_cycle = None;
    }

    /// Выполняет `seconds` секунд модели (160 МГц), с промоткой сна. Ошибка —
    /// только при неверной длительности; сбой прошивки — не ошибка: см.
    /// [`Mcu::fault`] и [`RunReport::halted`].
    pub fn run_for(&mut self, seconds: f64) -> Result<RunReport, McuError> {
        if !seconds.is_finite() || seconds < 0.0 {
            return Err(McuError::BadDuration(seconds));
        }
        self.target_cycles += seconds * CPU_HZ as f64;
        let target = self.target_cycles.round() as u64;
        Ok(self.run_until(target))
    }

    /// Выполняет `n` тактов.
    pub fn run_cycles(&mut self, n: u64) -> RunReport {
        let target = self.clk.cycle.saturating_add(n);
        self.target_cycles = target as f64;
        self.run_until(target)
    }

    /// Ровно одна инструкция (или вход в прерывание / промотка сна до события).
    pub fn step(&mut self) -> RunReport {
        let target = self.clk.cycle + 1;
        let r = self.run_until(target);
        self.target_cycles = self.target_cycles.max(self.clk.cycle as f64);
        r
    }

    fn account(&mut self, cycles: u64, active: bool) {
        if active {
            self.active_total += cycles;
            self.win_active += cycles;
        } else {
            self.sleep_total += cycles;
        }
        self.win_total += cycles;
        if self.win_total >= self.window_cycles {
            self.last_load = Some(self.win_active as f64 / self.win_total as f64);
            self.win_active = 0;
            self.win_total = 0;
        }
    }

    fn idle_until(&mut self, until: u64) {
        let n = until.saturating_sub(self.clk.cycle);
        self.clk.cycle += n;
        self.account(n, false);
    }

    fn run_until(&mut self, target: u64) -> RunReport {
        self.run_until_with_execution(target, true)
    }

    fn run_until_with_execution(&mut self, target: u64, executing: bool) -> RunReport {
        let (c0, i0) = (self.clk.cycle, self.clk.instret);
        while self.clk.cycle < target {
            let now = self.clk.cycle;
            if let Some(e) = self.bus.p.wdt.expiry()
                && e <= now
            {
                self.chip_reset(ResetReason::Watchdog);
                continue;
            }
            if !executing {
                let until = self.bus.p.wdt.expiry().unwrap_or(u64::MAX).min(target);
                self.idle_until(until);
                continue;
            }
            match self.halt {
                Some(HaltReason::Fault(_)) => {
                    // Ядро стоит, сторож идёт.
                    let until = self.bus.p.wdt.expiry().unwrap_or(u64::MAX).min(target);
                    self.idle_until(until);
                    continue;
                }
                Some(_) => {
                    self.idle_until(target);
                    break;
                }
                None => {}
            }
            self.bus.dirty = false;
            self.cpu.csr.mip = self.bus.p.mip(now);
            let next_event = self.bus.p.next_event(now);
            if self.cpu.sleeping {
                if self.cpu.csr.mip & self.cpu.csr.mie != 0 {
                    self.cpu.sleeping = false; // пробуждение даже при mstatus.MIE = 0
                } else {
                    self.idle_until(next_event.min(target));
                    continue;
                }
            }
            let before = self.clk.cycle;
            self.cpu.take_interrupt(&mut self.clk);
            let budget = next_event.min(target);
            let exit = self.cpu.run_batch(&mut self.bus, &mut self.clk, budget);
            let spent = self.clk.cycle - before;
            self.account(spent, true);
            if let Some(req) = self.bus.p.request.take() {
                match req {
                    Request::Halt(code) => {
                        self.halt = Some(HaltReason::Exit(code));
                        self.halt_cycle = Some(self.clk.cycle);
                    }
                    Request::SoftwareReset => self.chip_reset(ResetReason::Software),
                    Request::FirmwareFault { cause, pc, tval } => {
                        self.set_fault(Fault {
                            kind: FaultKind::from_cause(cause),
                            cause,
                            pc,
                            tval,
                            source: FaultSource::Firmware,
                            time_s: self.time(),
                        });
                    }
                }
                continue;
            }
            match exit {
                Exit::Breakpoint => {
                    self.halt = Some(HaltReason::Breakpoint { pc: self.cpu.pc });
                    self.halt_cycle = Some(self.clk.cycle);
                }
                Exit::Fault => {
                    if let Some(f) = self.cpu.fault {
                        self.set_fault(Fault {
                            kind: FaultKind::from_cause(f.cause),
                            cause: f.cause,
                            pc: f.pc,
                            tval: f.tval,
                            source: FaultSource::TrapLoop {
                                first_cause: f.first_cause,
                                first_pc: f.first_pc,
                            },
                            time_s: self.time(),
                        });
                    }
                }
                Exit::Budget | Exit::Reeval | Exit::Sleep => {}
            }
        }
        RunReport {
            cycles: self.clk.cycle - c0,
            instructions: self.clk.instret - i0,
            halted: self.halt.is_some(),
        }
    }

    fn set_fault(&mut self, f: Fault) {
        if self.bus.p.host_timestamps {
            self.host_faults.push((self.clk.cycle, f.clone()));
        }
        self.fault = Some(f.clone());
        self.halt = Some(HaltReason::Fault(f));
        self.halt_cycle = Some(self.clk.cycle);
    }

    // ---------- время и счётчики ----------

    /// Время модели, с.
    pub fn time(&self) -> f64 {
        self.clk.cycle as f64 / CPU_HZ as f64
    }

    /// Такты с момента создания (включая сон).
    pub fn cycles(&self) -> u64 {
        self.clk.cycle
    }

    /// Выполненные инструкции.
    pub fn instret(&self) -> u64 {
        self.clk.instret
    }

    /// Загрузка ЦП: доля тактов вне сна `wfi` за последнее законченное окно
    /// (по умолчанию 10 мс модели); до первого окна — за то, что прошло.
    pub fn cpu_load(&self) -> f64 {
        self.last_load.unwrap_or_else(|| {
            if self.win_total == 0 {
                0.0
            } else {
                self.win_active as f64 / self.win_total as f64
            }
        })
    }

    /// Такты вне сна `wfi` с момента создания (для своей загрузки ЦП по окну).
    pub fn active_cycles(&self) -> u64 {
        self.active_total
    }

    /// Загрузка ЦП с момента создания.
    pub fn cpu_load_total(&self) -> f64 {
        let t = self.active_total + self.sleep_total;
        if t == 0 {
            0.0
        } else {
            self.active_total as f64 / t as f64
        }
    }

    /// Окно усреднения [`Mcu::cpu_load`], с.
    pub fn set_load_window(&mut self, seconds: f64) {
        self.window_cycles = ((seconds * CPU_HZ as f64) as u64).max(1);
    }

    // ---------- входы и выходы ----------

    /// Напряжение на входе АЦП (канал 0..=7), В. Неверный канал игнорируется.
    pub fn set_adc_volts(&mut self, ch: usize, v: f64) {
        if let Some(slot) = self.bus.p.adc.volts.get_mut(ch) {
            *slot = v;
            if let Some(mask) = &mut self.bus.p.adc_inputs {
                *mask |= 1 << ch;
            }
        }
    }

    /// Код, который АЦП выдаст для канала сейчас.
    pub fn adc_code(&self, ch: usize) -> u32 {
        self.bus.p.adc.volts.get(ch).map_or(0, |&v| adc::code(v))
    }

    /// Уровень, подаваемый хостом на вывод GPIO (если вывод — вход).
    pub fn set_gpio_input(&mut self, pin: usize, high: bool) {
        if pin < 32 {
            let m = 1u32 << pin;
            let g: &mut gpio::Gpio = &mut self.bus.p.gpio;
            g.input = if high { g.input | m } else { g.input & !m };
        }
    }

    /// Вывод настроен на выход и выдаёт высокий уровень.
    pub fn gpio_output(&self, pin: usize) -> bool {
        pin < 32 && {
            let g = &self.bus.p.gpio;
            (g.dir & g.out) >> pin & 1 != 0
        }
    }

    /// Уровни на всех 32 выводах.
    pub fn gpio_levels(&self) -> u32 {
        self.bus.p.gpio.levels()
    }

    /// Доля заполнения ШИМ 0..=1 (выключенный канал — 0).
    pub fn pwm_duty(&self, ch: usize) -> f64 {
        self.bus.p.pwm.duty(ch)
    }

    /// Частота несущей ШИМ, Гц.
    pub fn pwm_freq_hz(&self, ch: usize) -> u32 {
        self.bus
            .p
            .pwm
            .ch
            .get(ch)
            .map_or(pwm::DEFAULT_FREQ_HZ, |c| c.freq_hz)
    }

    /// Включает отметки MMIO для новых выходов. Вызывать сразу после загрузки.
    pub fn enable_host_timestamps(&mut self) {
        self.bus.p.host_timestamps = true;
    }

    /// Для общей платы чтение АЦП без входа является ошибкой доступа.
    /// Вызывать до подачи напряжений. В прежнем интерфейсе сохраняется исходный ABI.
    pub fn require_adc_inputs(&mut self) {
        self.bus.p.adc_inputs = Some(0);
    }

    pub fn disconnect_adc(&mut self, channel: u32) {
        if let Some(mask) = &mut self.bus.p.adc_inputs
            && let Some(bit) = 1u8.checked_shl(channel)
        {
            *mask &= !bit;
        }
    }

    pub fn take_timed_uart(&mut self) -> Vec<(u64, u8)> {
        std::mem::take(&mut self.bus.p.uart_tx_cycles)
            .into_iter()
            .zip(std::mem::take(&mut self.bus.p.uart.tx))
            .collect()
    }

    pub fn take_timed_packets(&mut self) -> Vec<(u64, Vec<u8>)> {
        self.bus
            .p
            .radio_tx_cycles
            .drain(..)
            .zip(self.bus.p.radio.tx_fifo.drain(..))
            .collect()
    }

    pub fn take_timed_resets(&mut self) -> Vec<(u64, ResetReason)> {
        std::mem::take(&mut self.host_resets)
    }

    pub fn take_timed_faults(&mut self) -> Vec<(u64, Fault)> {
        std::mem::take(&mut self.host_faults)
    }

    /// Такт остановки по SYSCTRL.HALT, точке останова или сбою.
    pub fn halt_cycle(&self) -> Option<u64> {
        self.halt_cycle
    }

    pub fn gpio_drive(&self, pin: u32) -> Option<bool> {
        let gpio = &self.bus.p.gpio;
        let mask = 1u32.checked_shl(pin)?;
        (gpio.dir & mask != 0).then_some(gpio.out & mask != 0)
    }

    pub fn pwm_enabled(&self, channel: usize) -> bool {
        self.bus.p.pwm.ch.get(channel).is_some_and(|c| c.enabled)
    }

    /// Часы продолжают идти, включая сторож, пока внешняя причина удерживает ядро.
    pub fn run_with_paused_core(&mut self, target: u64) -> RunReport {
        self.target_cycles = target as f64;
        self.run_until_with_execution(target, false)
    }

    /// Интервал без питания. Аппаратные блоки и CPU не работают.
    pub fn advance_unpowered(&mut self, target: u64) {
        self.idle_until(target);
        self.target_cycles = self.clk.cycle as f64;
    }

    /// Забрать накопленный вывод UART как текст (неверный UTF-8 заменяется).
    pub fn uart_take(&mut self) -> String {
        String::from_utf8_lossy(&self.uart_take_bytes()).into_owned()
    }

    /// Забрать накопленный вывод UART байтами.
    pub fn uart_take_bytes(&mut self) -> Vec<u8> {
        self.bus.p.uart_tx_cycles.clear();
        std::mem::take(&mut self.bus.p.uart.tx)
    }

    /// Передать байты в приёмник UART (FIFO 256 байт). Возвращает число принятых.
    pub fn uart_send(&mut self, bytes: &[u8]) -> usize {
        self.bus.p.uart.host_send(bytes)
    }

    /// Забрать пакеты, переданные радио.
    pub fn radio_take_tx(&mut self) -> Vec<Vec<u8>> {
        self.bus.p.radio_tx_cycles.clear();
        self.bus.p.radio.tx_fifo.drain(..).collect()
    }

    /// Доставить пакет из эфира с RSSI, дБм. `false` — пакет отброшен (очередь
    /// из 8 пакетов полна, пустой или длиннее 251 байта).
    pub fn radio_deliver(&mut self, data: &[u8], rssi_dbm: i8) -> bool {
        self.bus.p.radio.deliver(data, rssi_dbm)
    }

    // ---------- состояние ----------

    /// Последний сбой с момента сброса по питанию (остаётся и после сброса сторожем).
    pub fn fault(&self) -> Option<&Fault> {
        self.fault.as_ref()
    }

    /// Почему ядро стоит (`None` — работает или спит).
    pub fn halt_reason(&self) -> Option<&HaltReason> {
        self.halt.as_ref()
    }

    pub fn is_halted(&self) -> bool {
        self.halt.is_some()
    }

    /// Код выхода, если прошивка остановилась через SYSCTRL.HALT.
    pub fn exit_code(&self) -> Option<u32> {
        match self.halt {
            Some(HaltReason::Exit(c)) => Some(c),
            _ => None,
        }
    }

    /// Сбросов сторожем с момента включения питания.
    pub fn watchdog_resets(&self) -> u32 {
        self.bus.p.sys.wdt_resets
    }

    /// Причина последнего сброса.
    pub fn reset_reason(&self) -> ResetReason {
        self.bus.p.sys.reset_reason
    }

    /// Ядро спит в `wfi`.
    pub fn is_sleeping(&self) -> bool {
        self.cpu.sleeping
    }

    pub fn pc(&self) -> u32 {
        self.cpu.pc
    }

    /// Регистр x0..=x31.
    pub fn reg(&self, i: usize) -> u32 {
        self.cpu.reg(i)
    }

    /// Сколько прерываний с кодом `code` (3 — программное, 7 — таймер, 11 —
    /// внешнее) взято с момента создания.
    pub fn interrupt_count(&self, code: u32) -> u64 {
        self.cpu.interrupts_taken[(code & 15) as usize]
    }

    /// Сколько исключений с кодом `code` (`mcause`) взято с момента создания.
    pub fn exception_count(&self, code: u32) -> u64 {
        self.cpu.exceptions_taken[(code & 15) as usize]
    }

    /// Ядро (CSR и прочее) — для тестов и отладки.
    pub fn cpu(&self) -> &Cpu {
        &self.cpu
    }

    /// Адрес символа прошивки из `.symtab`.
    pub fn symbol(&self, name: &str) -> Option<u32> {
        self.symbols.get(name).copied()
    }

    /// Прочитать память (флеш, ОЗУ, блок конфигурации) без побочных эффектов.
    pub fn read_bytes(&self, addr: u32, len: usize) -> Option<Vec<u8>> {
        let regions: [(u32, &[u8]); 3] = [
            (RAM_BASE, &self.bus.ram[..]),
            (FLASH_BASE, &self.bus.flash),
            (crate::bus::CONFIG_BASE, &self.bus.config),
        ];
        for (base, mem) in regions {
            let off = addr.wrapping_sub(base) as usize;
            if off < mem.len() && off + len <= mem.len() {
                return Some(mem[off..off + len].to_vec());
            }
        }
        None
    }

    pub fn read_u32(&self, addr: u32) -> Option<u32> {
        self.read_bytes(addr, 4)
            .map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    }

    pub fn read_f32(&self, addr: u32) -> Option<f32> {
        self.read_u32(addr).map(f32::from_bits)
    }

    /// Прочитать переменную прошивки по имени символа.
    pub fn read_symbol_u32(&self, name: &str) -> Option<u32> {
        self.symbol(name).and_then(|a| self.read_u32(a))
    }

    /// Частота ядра, Гц.
    pub const fn cpu_hz() -> u64 {
        CPU_HZ
    }
}

/// Проверка, что тип можно передавать между потоками (ресурс Bevy и т. п.).
const _: fn() = || {
    fn assert_send<T: Send + Sync>() {}
    assert_send::<Mcu>();
};

/// Коды причин — для удобства хоста.
pub use cpu::cause as mcause;
