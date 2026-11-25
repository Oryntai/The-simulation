//! SYSREG часов и SYSTIMER ESP32-C3. TRM v1.4 §§6.2, 10, 16.

use crate::{
    BoardFault,
    esp_clock::{
        CPU_PER_CONF, EspClock, EspClockRates, QUARTERS_PER_NS, QUARTERS_PER_SECOND, SYSCLK_CONF,
        SYSTEM, XTAL_HZ,
    },
};

// TRM табл. 3.3-3 и ESP-IDF soc/esp32c3/register/soc/reg_base.h.
const SYSTIMER: u32 = 0x6002_3000;
const SPI2: u32 = 0x6002_4000;
const WORD: u32 = size_of::<u32>() as u32;
// system_reg.h: SYSTEM_PERIP_CLK_EN0, SYSTEM_PERIP_RST_EN0, SYSTEM_SYSTIMER_{CLK_EN,RST}.
const PERIP_CLK_EN0: u32 = SYSTEM + 0x010;
const PERIP_RST_EN0: u32 = SYSTEM + 0x018;
const SYSTIMER_CONTROL: u32 = 1 << 29;

// TRM таблица 10.6-1 и ESP-IDF soc/esp32c3/register/soc/systimer_reg.h.
const CONF: u32 = 0x00;
const UNIT_OP: u32 = 0x04;
const UNIT_LOAD_VALUE: u32 = 0x0c;
const TARGET_VALUE: u32 = 0x1c;
const TARGET_CONF: u32 = 0x34;
const UNIT_VALUE: u32 = 0x40;
const COMP_LOAD: u32 = 0x50;
const UNIT_LOAD: u32 = 0x5c;
const INT_ENA: u32 = 0x64;
const INT_RAW: u32 = 0x68;
const INT_CLR: u32 = 0x6c;
const INT_ST: u32 = 0x70;
const DATE: u32 = 0xfc;
// Поля CONF по systimer_reg.h и esp_hal_systimer/esp32c3/include/hal/systimer_ll.h.
// Для однопроцессорного C3 биты CORE1 не управляют счётчиками.
const REGISTER_CLOCK: u32 = 1 << 31;
const UNIT_ENABLE: [u32; 2] = [1 << 30, 1 << 29];
const STALL_ENABLE: [u32; 2] = [1 << 28, 1 << 26];
const COMP_ENABLE: [u32; 3] = [1 << 24, 1 << 23, 1 << 22];
const CORE1_RESET: u32 = 1 << 25;
const CONF_RESET: u32 = UNIT_ENABLE[0] | STALL_ENABLE[1] | CORE1_RESET;
const CONF_MASK: u32 = REGISTER_CLOCK
    | UNIT_ENABLE[0]
    | UNIT_ENABLE[1]
    | STALL_ENABLE[0]
    | STALL_ENABLE[1]
    | COMP_ENABLE[0]
    | COMP_ENABLE[1]
    | COMP_ENABLE[2];
// UNITn_OP, UNITn_LOAD, COMPn_LOAD, TARGETn_CONF, INT_* и DATE по systimer_reg.h.
const UPDATE: u32 = 1 << 30;
const VALUE_VALID: u32 = 1 << 29;
const LOAD: u32 = 1;
const SELECT_UNIT1: u32 = 1 << 31;
const PERIOD_MODE: u32 = 1 << 30;
const PERIOD_MASK: u32 = (1 << 26) - 1;
const INT_MASK: u32 = 0b111;
const DATE_RESET: u32 = 0x0200_6171;
// TRM §§10.3–10.4: два 52-битных счётчика, чередование делителей /2 и /3.
const COUNT_RANGE: u64 = 1 << 52;
const COUNT_MASK: u64 = COUNT_RANGE - 1;
const PAIR: u128 = 5 * (QUARTERS_PER_SECOND / XTAL_HZ) as u128;

/// Начальная фаза делителя CNT_CLK относительно нуля времени модели.
/// TRM §10.3 задаёт два делителя, поэтому порядок их старта задаётся явно.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SystimerPhase {
    FirstAfterTwoXtalCycles,
    FirstAfterThreeXtalCycles,
}

impl SystimerPhase {
    fn first(self) -> u128 {
        let xtal_cycles = match self {
            Self::FirstAfterTwoXtalCycles => 2,
            Self::FirstAfterThreeXtalCycles => 3,
        };
        xtal_cycles * u128::from(QUARTERS_PER_SECOND / XTAL_HZ)
    }

    fn ticks(self, time: u128) -> u128 {
        2 * (time / PAIR) + u128::from(time % PAIR >= self.first())
    }

    fn edge(self, tick: u128) -> u128 {
        let pair = (tick - 1) / 2;
        pair * PAIR + if tick % 2 == 0 { PAIR } else { self.first() }
    }
}

/// Цифровое дерево часов и системный таймер профиля холодной загрузки.
#[derive(Clone)]
pub struct EspSystem {
    clock: EspClock,
    phase: SystimerPhase,
    timer: Systimer,
    apb_enabled: bool,
    timer_reset: bool,
}

impl EspSystem {
    pub fn new(phase: SystimerPhase) -> Self {
        Self {
            clock: EspClock::default(),
            phase,
            timer: Systimer::default(),
            // system_reg.h: SYSTIMER_CLK_EN=1, SYSTIMER_RST=0 при аппаратном сбросе.
            apb_enabled: true,
            timer_reset: false,
        }
    }

    /// Холодный сброс регистров с сохранением шкалы хоста и фазы внешнего XTAL.
    pub fn reset(&mut self) {
        let time = self.clock.time;
        *self = Self::new(self.phase);
        self.clock.time = time;
    }

    pub fn clocks(&self) -> EspClockRates {
        self.clock.rates()
    }

    /// Задержка закрытия тактового затвора при WFI. None — затвор принудительно открыт.
    pub fn wait_gate_cycles(&self) -> Option<u8> {
        self.clock.wait_gate_cycles()
    }

    pub fn elapsed_quarter_ns(&self) -> u128 {
        self.clock.time.elapsed()
    }

    /// Такты, полностью помещающиеся до абсолютной отметки хоста в наносекундах.
    /// Пересчитать после изменения SYSREG. Остаток сохраняется в абсолютной шкале.
    pub fn cycles_before(&self, deadline_ns: u64) -> u64 {
        let end = u128::from(deadline_ns) * u128::from(QUARTERS_PER_NS);
        let available = end.saturating_sub(self.elapsed_quarter_ns());
        (available / u128::from(self.clocks().cpu_cycle_quarters())) as u64
    }

    /// Частоты берутся на входе в такт. Запись SYSREG в нём действует на следующий такт.
    /// `core_stalled` указывает останов ядра на всём интервале, включая WFI с закрытым затвором.
    pub fn advance_cpu(
        &mut self,
        cycles: u64,
        clock_at_start: EspClockRates,
        core_stalled: bool,
    ) -> Result<(), BoardFault> {
        let before = self.phase.ticks(self.elapsed_quarter_ns());
        self.clock
            .time
            .advance(cycles, clock_at_start.cpu_cycle_quarters())?;
        let ticks = self.phase.ticks(self.elapsed_quarter_ns()) - before;
        if !self.timer_reset && cycles != 0 {
            self.timer.advance(ticks, core_stalled);
        }
        Ok(())
    }

    /// Уровни трёх выходов SYSTIMER, включая состояние при закрытом затворе APB.
    /// Матрица 2.9 читает их по адресу SYSTIMER_INT_ST (источники ETS 37–39).
    pub fn interrupt_status(&self) -> u32 {
        self.timer.raw & self.timer.interrupt_enable
    }

    /// Такты CPU до выставления хотя бы одного разрешённого выхода SYSTIMER.
    /// Ноль означает, что выход уже выставлен. Матрица CPU проверяет свои маски отдельно.
    pub fn cycles_until_irq(&self, core_stalled: bool) -> Option<u64> {
        if self.interrupt_status() != 0 {
            return Some(0);
        }
        if self.timer_reset {
            return None;
        }
        let distance = self.timer.ticks_until_irq(core_stalled)?;
        if distance == 0 {
            return Some(0);
        }
        let now = self.elapsed_quarter_ns();
        let deadline = self
            .phase
            .edge(self.phase.ticks(now) + u128::from(distance));
        Some((deadline - now).div_ceil(u128::from(self.clocks().cpu_cycle_quarters())) as u64)
    }

    /// Бит SYSTIMER в общих регистрах SYSREG. Владелец остальных полей объединяет их на шине.
    pub fn shared_sysreg_mask(addr: u32) -> u32 {
        match addr {
            PERIP_CLK_EN0 | PERIP_RST_EN0 => SYSTIMER_CONTROL,
            _ => 0,
        }
    }

    /// Доступ к часам и таймеру, None означает чужой адрес.
    /// Для общих SYSREG передаётся только поле из `shared_sysreg_mask`.
    pub fn access(
        &mut self,
        pc: u32,
        addr: u32,
        size: u8,
        value: Option<u32>,
    ) -> Result<Option<u32>, BoardFault> {
        let in_timer = (SYSTIMER..SPI2).contains(&addr);
        let in_system = [CPU_PER_CONF, SYSCLK_CONF, PERIP_CLK_EN0, PERIP_RST_EN0]
            .contains(&(addr & !(WORD - 1)));
        if !in_timer && !in_system {
            return Ok(None);
        }
        let unsupported = || BoardFault::UnsupportedRegister {
            pc,
            addr,
            write: value.is_some(),
            size,
            block: "SYSREG часов и SYSTIMER ESP32-C3",
        };
        if size != WORD as u8 {
            return Err(unsupported());
        }
        crate::memory::check_data_alignment(pc, addr, size)?;
        let result = match addr {
            CPU_PER_CONF | SYSCLK_CONF => self.clock.access(addr, value)?,
            PERIP_CLK_EN0 | PERIP_RST_EN0 => self.access_timer_control(addr, value)?,
            _ => {
                if !self.apb_enabled || self.timer_reset {
                    return Err(BoardFault::Unsupported(
                        "доступ SYSTIMER при отключённом APB или активном сбросе".into(),
                    ));
                }
                self.timer
                    .access(pc, addr, value)?
                    .ok_or_else(unsupported)?
            }
        };
        Ok(Some(result))
    }

    fn access_timer_control(&mut self, addr: u32, value: Option<u32>) -> Result<u32, BoardFault> {
        let is_gate = addr == PERIP_CLK_EN0;
        let state = if is_gate {
            &mut self.apb_enabled
        } else {
            &mut self.timer_reset
        };
        if let Some(bits) = value {
            if bits & !SYSTIMER_CONTROL != 0 {
                return Err(BoardFault::Unsupported(
                    "переданы поля SYSREG других блоков".into(),
                ));
            }
            *state = bits & SYSTIMER_CONTROL != 0;
            if self.timer_reset {
                self.timer = Systimer::default();
            }
            Ok(0)
        } else {
            Ok(if *state { SYSTIMER_CONTROL } else { 0 })
        }
    }
}

#[derive(Clone, Default)]
struct Counter {
    count: u64,
    load: u64,
    snapshot: u64,
    valid: bool,
}

#[derive(Clone, Default)]
struct Comparator {
    config: u32,
    load: u64,
    target: u64,
    period: u64,
    remaining: Option<u64>,
}

impl Comparator {
    fn unit(&self) -> usize {
        usize::from(self.config & SELECT_UNIT1 != 0)
    }

    fn periodic(&self) -> bool {
        self.config & PERIOD_MODE != 0
    }

    fn validate(&self, enabled: bool) -> Result<(), BoardFault> {
        if enabled && self.periodic() && self.period == 0 {
            return Err(BoardFault::Misconfigured(
                "активный SYSTIMER COMP с нулевым загруженным периодом".into(),
            ));
        }
        Ok(())
    }

    fn arm(&mut self, enabled: bool, now: u64) {
        self.remaining = enabled.then(|| {
            if self.periodic() {
                return self.period;
            }
            // TRM §10.4.2 и табл. 10.4-2: ближняя просроченная цель немедленна,
            // дальняя ждёт оборота. QEMU Espressif hw/timer/esp_systimer.c
            // использует эту границу и при переходе счётчика между половинами диапазона.
            if now >= self.target && now - self.target < COUNT_RANGE / 2 - 1 {
                0
            } else {
                self.target.wrapping_sub(now) & COUNT_MASK
            }
        });
    }

    fn advance(&mut self, ticks: u128) -> bool {
        let Some(remaining) = self.remaining.map(u128::from) else {
            return false;
        };
        if ticks < remaining {
            self.remaining = Some((remaining - ticks) as u64);
            return false;
        }
        self.remaining = self
            .periodic()
            .then(|| self.period - ((ticks - remaining) % u128::from(self.period)) as u64);
        true
    }
}

#[derive(Clone)]
struct Systimer {
    conf: u32,
    counters: [Counter; UNIT_ENABLE.len()],
    comparators: [Comparator; COMP_ENABLE.len()],
    interrupt_enable: u32,
    raw: u32,
    date: u32,
}

impl Default for Systimer {
    fn default() -> Self {
        Self {
            conf: CONF_RESET,
            counters: Default::default(),
            comparators: Default::default(),
            interrupt_enable: 0,
            raw: 0,
            date: DATE_RESET,
        }
    }
}

impl Systimer {
    fn counting(&self, unit: usize, core_stalled: bool) -> bool {
        self.conf & UNIT_ENABLE[unit] != 0 && !(core_stalled && self.conf & STALL_ENABLE[unit] != 0)
    }

    fn ticks_until_irq(&self, core_stalled: bool) -> Option<u64> {
        self.comparators
            .iter()
            .enumerate()
            .filter(|(i, comp)| {
                self.interrupt_enable & (1 << i) != 0 && self.counting(comp.unit(), core_stalled)
            })
            .filter_map(|(_, comp)| comp.remaining)
            .min()
    }

    fn advance(&mut self, ticks: u128, core_stalled: bool) {
        let counting = std::array::from_fn::<_, { UNIT_ENABLE.len() }, _>(|unit| {
            self.counting(unit, core_stalled)
        });
        for (counter, running) in self.counters.iter_mut().zip(counting) {
            if running {
                counter.count =
                    ((u128::from(counter.count) + ticks) & u128::from(COUNT_MASK)) as u64;
            }
        }
        for (i, comp) in self.comparators.iter_mut().enumerate() {
            if counting[comp.unit()] && comp.advance(ticks) {
                self.raw |= 1 << i;
            }
        }
    }

    fn rearm(&mut self, i: usize) {
        let comp = &mut self.comparators[i];
        comp.arm(
            self.conf & COMP_ENABLE[i] != 0,
            self.counters[comp.unit()].count,
        );
    }

    fn set_conf(&mut self, value: u32) -> Result<(), BoardFault> {
        for (i, &enable) in COMP_ENABLE.iter().enumerate() {
            self.comparators[i].validate(value & enable != 0)?;
        }
        let changed = self.conf ^ value;
        self.conf = (value & CONF_MASK) | CORE1_RESET;
        for (i, &enable) in COMP_ENABLE.iter().enumerate() {
            if changed & enable != 0 {
                self.rearm(i);
            }
        }
        self.advance(0, false);
        Ok(())
    }

    fn configure_comp(
        &mut self,
        i: usize,
        value: Option<u32>,
        commit: bool,
    ) -> Result<u32, BoardFault> {
        let current = &self.comparators[i];
        let Some(bits) = value else {
            return Ok(if commit { 0 } else { current.config });
        };
        let mut candidate = current.clone();
        if commit {
            if bits & LOAD == 0 {
                return Ok(0);
            }
            candidate.period = u64::from(candidate.config & PERIOD_MASK);
            candidate.target = candidate.load;
        } else {
            candidate.config = bits & (SELECT_UNIT1 | PERIOD_MODE | PERIOD_MASK);
        }
        let enable = self.conf & COMP_ENABLE[i] != 0;
        candidate.validate(enable)?;
        let rearm =
            commit || (current.config ^ candidate.config) & (SELECT_UNIT1 | PERIOD_MODE) != 0;
        self.comparators[i] = candidate;
        if rearm {
            self.rearm(i);
            self.advance(0, false);
        }
        Ok(0)
    }

    fn reload_counter(&mut self, unit: usize, value: Option<u32>) {
        if value.is_some_and(|bits| bits & LOAD != 0) {
            self.counters[unit].count = self.counters[unit].load;
            // TRM §10.4.1: загрузка меняет текущее значение сразу.
            // Для уже запущенного периода новая база отсчёта совпадает с моделью
            // esp_systimer_load_counter в QEMU Espressif hw/timer/esp_systimer.c.
            for i in 0..self.comparators.len() {
                let comp = &self.comparators[i];
                if comp.unit() == unit && comp.remaining.is_some() {
                    self.rearm(i);
                }
            }
            self.advance(0, false);
        }
    }

    fn access(
        &mut self,
        pc: u32,
        addr: u32,
        value: Option<u32>,
    ) -> Result<Option<u32>, BoardFault> {
        let offset = addr - SYSTIMER;
        let result = match offset {
            CONF => {
                if let Some(bits) = value {
                    self.set_conf(bits)?;
                }
                self.conf
            }
            UNIT_OP..UNIT_LOAD_VALUE => {
                let unit = &mut self.counters[((offset - UNIT_OP) / WORD) as usize];
                if let Some(bits) = value {
                    if bits & VALUE_VALID != 0 {
                        unit.valid = false;
                    }
                    if bits & UPDATE != 0 {
                        unit.snapshot = unit.count;
                        unit.valid = true;
                    }
                }
                if unit.valid { VALUE_VALID } else { 0 }
            }
            UNIT_LOAD_VALUE..TARGET_VALUE => {
                let (index, high) = split_register(offset, UNIT_LOAD_VALUE);
                access_half(&mut self.counters[index].load, high, value)
            }
            TARGET_VALUE..TARGET_CONF => {
                let (index, high) = split_register(offset, TARGET_VALUE);
                access_half(&mut self.comparators[index].load, high, value)
            }
            TARGET_CONF..UNIT_VALUE => {
                self.configure_comp(((offset - TARGET_CONF) / WORD) as usize, value, false)?
            }
            UNIT_VALUE..COMP_LOAD => {
                if value.is_some() {
                    return Err(BoardFault::WriteToReadOnly { pc, addr });
                }
                let (index, high) = split_register(offset, UNIT_VALUE);
                access_half(&mut self.counters[index].snapshot, high, None)
            }
            COMP_LOAD..UNIT_LOAD => {
                self.configure_comp(((offset - COMP_LOAD) / WORD) as usize, value, true)?
            }
            UNIT_LOAD..INT_ENA => {
                self.reload_counter(((offset - UNIT_LOAD) / WORD) as usize, value);
                0
            }
            INT_ENA => {
                if let Some(bits) = value {
                    self.interrupt_enable = bits & INT_MASK;
                }
                self.interrupt_enable
            }
            INT_RAW | INT_CLR => {
                // INT_RAW: WTC/SS, INT_CLR: WT. Источник устанавливает RAW при совпадении.
                self.raw &= !(value.unwrap_or(0) & INT_MASK);
                if offset == INT_RAW { self.raw } else { 0 }
            }
            INT_ST => {
                if value.is_some() {
                    return Err(BoardFault::WriteToReadOnly { pc, addr });
                }
                self.raw & self.interrupt_enable
            }
            DATE => {
                if let Some(bits) = value {
                    self.date = bits;
                }
                self.date
            }
            _ => return Ok(None),
        };
        Ok(Some(if value.is_some() { 0 } else { result }))
    }
}

// TRM §10.6: сначала HI, затем LO, каждое поле занимает одно слово.
fn split_register(offset: u32, base: u32) -> (usize, bool) {
    let word_index = (offset - base) / WORD;
    ((word_index / 2) as usize, word_index % 2 == 0)
}

fn access_half(register: &mut u64, high: bool, value: Option<u32>) -> u32 {
    let shift = if high { u32::BITS } else { 0 };
    let mask = u64::from(u32::MAX) << shift;
    if let Some(bits) = value {
        *register = ((*register & !mask) | (u64::from(bits) << shift)) & COUNT_MASK;
    }
    (*register >> shift) as u32
}

// Подключение шины отложено до 2.10. В стенде используем саму матрицу 2.9.
#[cfg(test)]
#[path = "esp_irq.rs"]
mod irq_matrix;

#[cfg(test)]
#[test]
fn три_реальных_выхода_таймера_маршрутизируются_матрицей() {
    // TRM §8.5 и ESP-IDF soc/esp32c3/include/soc/interrupts.h.
    const MATRIX: u32 = 0x600c_2000;
    const ENABLE: u32 = MATRIX + 0x104;
    const PRIORITY: u32 = MATRIX + 0x114;
    const FIRST_SOURCE: u32 = 37;
    // PC внутри SRAM1 по TRM §3.3.2. Для стенда выбраны три разные линии.
    const PC: u32 = 0x4038_0000;
    const LINES: [u32; 3] = [5, 6, 7];

    let mut system = EspSystem::new(SystimerPhase::FirstAfterThreeXtalCycles);
    let mut irq = irq_matrix::EspIrq::default();
    let write = |system: &mut EspSystem, offset, value| {
        assert_eq!(
            system.access(PC, SYSTIMER + offset, WORD as u8, Some(value)),
            Ok(Some(0))
        );
    };
    let read_status = |addr| {
        assert_eq!(addr, SYSTIMER + INT_ST);
        Ok(system.interrupt_status())
    };
    for (i, line) in LINES.into_iter().enumerate() {
        irq.access(
            PC,
            MATRIX + (FIRST_SOURCE + i as u32) * WORD,
            WORD as u8,
            Some(line),
        )
        .unwrap();
        irq.access(PC, PRIORITY + line * WORD, WORD as u8, Some(1))
            .unwrap();
    }
    let mask = LINES.iter().fold(0, |bits, line| bits | (1 << line));
    assert!(irq_matrix::EspIrq::handles(ENABLE));
    irq.access(PC, ENABLE, WORD as u8, Some(mask)).unwrap();
    irq.sample(PC, None, read_status).unwrap();
    assert_eq!(irq.next(), None);
    for (i, &enable) in COMP_ENABLE.iter().enumerate() {
        // При холодном XTAL/2 два такта CPU пересекают первую границу счётчика.
        let now = system.timer.counters[0].count;
        write(
            &mut system,
            TARGET_VALUE + (2 * i as u32 + 1) * WORD,
            (now + 1) as u32,
        );
        write(&mut system, COMP_LOAD + i as u32 * WORD, LOAD);
        write(&mut system, INT_ENA, 1 << i);
        write(&mut system, CONF, UNIT_ENABLE[0] | enable);
        assert_eq!(system.interrupt_status(), 0);
        let cycles = system.cycles_until_irq(false).unwrap();
        system.advance_cpu(cycles, system.clocks(), false).unwrap();
        irq.sample(PC, None, |addr| {
            system
                .access(PC, addr, WORD as u8, None)
                .map(Option::unwrap)
        })
        .unwrap();
        assert_eq!(irq.next(), Some(LINES[i]));
        write(&mut system, INT_CLR, 1 << i);
        irq.sample(PC, None, |addr| {
            system
                .access(PC, addr, WORD as u8, None)
                .map(Option::unwrap)
        })
        .unwrap();
        assert_eq!(irq.next(), None);
    }
}
