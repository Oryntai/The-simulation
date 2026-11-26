//! Матрица прерываний ESP32-C3. TRM v1.4 §§1.5, 8.3–8.5 и 16.3.3.

use crate::BoardFault;

// ESP-IDF soc/esp32c3/register/soc/{reg_base,interrupt_core0_reg,system_reg}.h.
const MATRIX: u32 = 0x600c_2000;
const FROM_CPU: u32 = 0x600c_0028;
const SOURCE_COUNT: usize = 62;
const STANDARD_LINES: u32 = !1; // TRM §1.5.2: линия 0 зарезервирована.
const STAT_LOW: u32 = MATRIX + 0xf8;
const STAT_HIGH: u32 = MATRIX + 0xfc;
const ENABLE: u32 = MATRIX + 0x104;
const TYPE: u32 = MATRIX + 0x108;
const CLEAR: u32 = MATRIX + 0x10c;
const PENDING: u32 = MATRIX + 0x110;
const PRIORITY: u32 = MATRIX + 0x114;
const THRESHOLD: u32 = MATRIX + 0x194;
const WORD: u8 = size_of::<u32>() as u8;

// ESP-IDF soc/esp32c3/include/soc/interrupts.h, номера ETS_*_INTR_SOURCE.
const SYSTIMER_FIRST: u32 = 37;
const SYSTIMER_MASK: u64 = 0b111 << SYSTIMER_FIRST;
const SOFTWARE_FIRST: u32 = 50;
const SOFTWARE_MASK: u64 = 0b1111 << SOFTWARE_FIRST;
const RTC_CORE: u32 = 27;
const RTC_CORE_MASK: u64 = 1 << RTC_CORE;
const ASSIST_DEBUG_MASK: u64 = 1 << 54; // ETS_ASSIST_DEBUG_INTR_SOURCE.
const CACHE_ILLEGAL_MASK: u64 = 1 << 36; // ETS_CACHE_IA_INTR_SOURCE.
const CACHE_ACCESS_MASK: u64 = 1 << 61; // ETS_CACHE_CORE0_ACS_INTR_SOURCE.
const PMS_MASK: u64 = 0b111 << 56; // ETS_CORE0_{IRAM0,DRAM0,PIF}_PMS_INTR_SOURCE.

// ESP-IDF soc/esp32c3/register/soc/{reg_base,uart_reg,spi_reg,i2c_reg,timer_group_reg}.h.
// Адреса INT_ST и маски отдельных источников TIMG.
const PERIPHERALS: [(u32, u32, u32); 8] = [
    (19, 0x6002_4040, u32::MAX), // SPI2 DMA_INT_ST.
    (21, 0x6000_0008, u32::MAX), // UART0 INT_ST.
    (22, 0x6001_0008, u32::MAX), // UART1 INT_ST.
    (29, 0x6001_302c, u32::MAX), // I2C_EXT0 INT_STATUS.
    (32, 0x6001_f078, 1 << 0),   // TIMG0 T0.
    (33, 0x6001_f078, 1 << 1),   // TIMG0 WDT.
    (34, 0x6002_0078, 1 << 0),   // TIMG1 T0.
    (35, 0x6002_0078, 1 << 1),   // TIMG1 WDT.
];
// ESP-IDF soc/esp32c3/register/soc/systimer_reg.h, TARGET[0–2]_INT_ST.
pub(super) const SYSTIMER_STATUS: u32 = 0x6002_3070;

#[derive(Clone)]
pub(crate) struct EspIrq {
    map: [u8; SOURCE_COUNT],
    enabled: u32,
    edge: u32,
    clear: u32,
    priority: [u8; u32::BITS as usize],
    threshold: u8,
    software: u64,
    sources: u64,
    mapped_sources: u64,
    requested_sources: u64,
    routed: u32,
    latched: u32,
    claimed: u32,
}

impl Default for EspIrq {
    fn default() -> Self {
        // TRM §8.5: поля регистров матрицы сброшены в ноль.
        Self {
            map: [0; SOURCE_COUNT],
            enabled: 0,
            edge: 0,
            clear: 0,
            priority: [0; u32::BITS as usize],
            threshold: 0,
            software: 0,
            sources: 0,
            mapped_sources: 0,
            requested_sources: 0,
            routed: 0,
            latched: 0,
            claimed: 0,
        }
    }
}

impl EspIrq {
    pub(crate) fn handles(addr: u32) -> bool {
        (MATRIX..=THRESHOLD + u32::from(WORD) - 1).contains(&addr)
            || (FROM_CPU..FROM_CPU + 4 * u32::from(WORD)).contains(&addr)
    }

    #[cfg(test)]
    pub(crate) fn sample(
        &mut self,
        pc: u32,
        status_addr: Option<u32>,
        read: impl FnMut(u32) -> Result<u32, BoardFault>,
    ) -> Result<(), BoardFault> {
        self.sample_with_boot_sources(pc, status_addr, None, None, None, read)
    }

    /// Выборка линий от уже подключённых блоков на границе инструкций.
    /// Чтения INT_ST побочных эффектов не имеют. Ошибку шины передаём вызывающему.
    pub(crate) fn sample_with_boot_sources(
        &mut self,
        pc: u32,
        status_addr: Option<u32>,
        rtc_core: Option<bool>,
        assist: Option<bool>,
        memory: Option<((bool, bool), u64)>,
        mut read: impl FnMut(u32) -> Result<u32, BoardFault>,
    ) -> Result<(), BoardFault> {
        let observed = match status_addr {
            Some(STAT_LOW) => u64::from(u32::MAX),
            Some(STAT_HIGH) => u64::MAX << u32::BITS,
            _ => 0,
        };
        let needed = self.mapped_sources | observed;
        let mut active = self.software;
        if needed == 0 {
            self.set_sources(active);
            return Ok(());
        }
        let mut known = SOFTWARE_MASK | SYSTIMER_MASK;
        if let Some(rtc_core) = rtc_core {
            known |= RTC_CORE_MASK;
            if rtc_core {
                active |= RTC_CORE_MASK;
            }
        }
        if let Some(level) = assist {
            known |= ASSIST_DEBUG_MASK;
            if level {
                active |= ASSIST_DEBUG_MASK;
            }
        }
        if let Some(((illegal, access), pms)) = memory {
            known |= CACHE_ILLEGAL_MASK | CACHE_ACCESS_MASK | PMS_MASK;
            active |= pms;
            if illegal {
                active |= CACHE_ILLEGAL_MASK;
            }
            if access {
                active |= CACHE_ACCESS_MASK;
            }
        }
        for (source, register, mask) in PERIPHERALS {
            let bit = 1 << source;
            known |= bit;
            if needed & bit != 0 && read(register)? & mask != 0 {
                active |= bit;
            }
        }
        let missing = self.requested_sources & !known;
        if missing != 0 {
            let source = missing.trailing_zeros();
            return Err(BoardFault::UnsupportedRegister {
                pc,
                addr: MATRIX + source * u32::from(WORD),
                write: false,
                size: WORD,
                block: "линия источника матрицы IRQ не подключена",
            });
        }
        if needed & SYSTIMER_MASK != 0 {
            match read(SYSTIMER_STATUS) {
                Ok(value) => active |= u64::from(value & 0b111) << SYSTIMER_FIRST,
                // Отключённый маршрут можно настроить до подключения блока.
                Err(BoardFault::UnsupportedRegister { .. })
                    if self.requested_sources & SYSTIMER_MASK == 0 => {}
                Err(error) => return Err(error),
            }
        }
        self.set_sources(active);
        Ok(())
    }

    fn set_sources(&mut self, active: u64) {
        self.sources = active;
        let mut bits = active;
        let mut lines = 0;
        while bits != 0 {
            let source = bits.trailing_zeros() as usize;
            bits &= bits - 1;
            lines |= 1 << self.map[source];
        }
        lines &= STANDARD_LINES;
        // TRM §1.5.2: детектор фронта стоит на входе CPU после объединения источников.
        self.latched |= lines & !self.routed & self.edge & self.enabled;
        self.routed = lines;
    }

    pub(crate) fn next(&self) -> Option<u32> {
        let mut candidates = self.pending();
        let mut best = None;
        let mut priority = 0;
        // §1.5.2: при равном приоритете побеждает меньший номер.
        while candidates != 0 {
            let line = candidates.trailing_zeros();
            candidates &= candidates - 1;
            let level = self.priority[line as usize];
            if level > priority {
                best = Some(line);
                priority = level;
            }
        }
        best
    }

    pub(crate) fn claim(&mut self, line: u32) {
        self.claimed |= (1 << line) & self.edge;
    }

    fn pending(&self) -> u32 {
        let mut candidates = ((self.routed & !self.edge) | self.latched) & self.enabled;
        let mut pending = 0;
        while candidates != 0 {
            let line = candidates.trailing_zeros();
            candidates &= candidates - 1;
            let level = self.priority[line as usize];
            if level != 0 && level >= self.threshold {
                pending |= 1 << line;
            }
        }
        pending
    }

    pub(crate) fn access(
        &mut self,
        pc: u32,
        addr: u32,
        size: u8,
        value: Option<u32>,
    ) -> Result<u32, BoardFault> {
        let unsupported = || BoardFault::UnsupportedRegister {
            pc,
            addr,
            write: value.is_some(),
            size,
            block: "матрица IRQ ESP32-C3",
        };
        crate::memory::check_data_alignment(pc, addr, size)?;
        if size != WORD {
            return Err(unsupported());
        }

        let word = u32::from(WORD);
        let source = (addr.wrapping_sub(MATRIX) / word) as usize;
        let mut changed_routes = false;
        let old = match addr {
            _ if source < SOURCE_COUNT => {
                let old = self.map[source];
                if let Some(v) = value {
                    // *_MAP: пять младших битов, ноль отключает маршрут.
                    self.map[source] = (v & 31) as u8;
                    changed_routes = old != self.map[source];
                }
                u32::from(old)
            }
            STAT_LOW | STAT_HIGH | PENDING if value.is_some() => {
                return Err(BoardFault::WriteToReadOnly { pc, addr });
            }
            STAT_LOW => self.sources as u32,
            STAT_HIGH => (self.sources >> u32::BITS) as u32,
            ENABLE => {
                let old = self.enabled;
                if let Some(v) = value {
                    self.enabled = v & STANDARD_LINES;
                    changed_routes = old != self.enabled;
                }
                old
            }
            TYPE => {
                let old = self.edge;
                if let Some(v) = value {
                    self.edge = v & STANDARD_LINES;
                    self.claimed &= self.edge;
                    self.latched &= self.edge;
                }
                old
            }
            CLEAR => {
                let old = self.clear;
                if let Some(v) = value {
                    self.clear = v & STANDARD_LINES;
                    // TRM §1.5.2: последовательность 1→0 подтверждает принятый фронт.
                    // Для сброса непринятого фронта сначала отключается линия.
                    let acknowledged = old & !self.clear & (self.claimed | !self.enabled);
                    self.latched &= !acknowledged;
                    self.claimed &= !acknowledged;
                }
                old
            }
            PENDING => self.pending(),
            PRIORITY..THRESHOLD => {
                let priority = &mut self.priority[((addr - PRIORITY) / word) as usize];
                let old = *priority;
                if let Some(v) = value {
                    *priority = (v & 15) as u8; // CPU_INT_PRI_n[3:0], §8.5.
                }
                u32::from(old)
            }
            THRESHOLD => {
                let old = self.threshold;
                if let Some(v) = value {
                    self.threshold = (v & 15) as u8; // CPU_INT_THRESH[3:0], §8.5.
                }
                u32::from(old)
            }
            _ if (FROM_CPU..FROM_CPU + 4 * word).contains(&addr) => {
                let bit = 1 << (SOFTWARE_FIRST + (addr - FROM_CPU) / word);
                let old = u32::from(self.software & bit != 0);
                if let Some(v) = value {
                    // SYSTEM_CPU_INTR_FROM_CPU_n[0], уровень до программного сброса.
                    self.software &= !bit;
                    self.software |= (v as u64 & 1) * bit;
                }
                old
            }
            _ => return Err(unsupported()),
        };
        if changed_routes {
            self.mapped_sources = 0;
            self.requested_sources = 0;
            for (source, &line) in self.map.iter().enumerate() {
                let bit = 1 << source;
                if line != 0 {
                    self.mapped_sources |= bit;
                }
                if self.enabled & (1 << line) != 0 {
                    self.requested_sources |= bit;
                }
            }
        }
        Ok(old)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const PC: u32 = 0x4038_0000; // TRM §3.3.2, начало SRAM1 на IBUS.
    const LINE: u32 = 5; // Линия стенда из диапазона 1–31, TRM §1.5.
    const BIT: u32 = 1 << LINE;

    fn write(irq: &mut EspIrq, addr: u32, value: u32) {
        irq.access(PC, addr, WORD, Some(value)).unwrap();
    }

    fn read(irq: &mut EspIrq, addr: u32) -> u32 {
        irq.access(PC, addr, WORD, None).unwrap()
    }

    fn systimer(irq: &mut EspIrq, status: u32) {
        irq.sample(PC, None, |addr| {
            assert_eq!(addr, SYSTIMER_STATUS);
            Ok(status)
        })
        .unwrap();
    }

    #[test]
    fn три_входа_systimer_уровень_и_защёлка_фронта() {
        let mut irq = EspIrq::default();
        for source in SYSTIMER_FIRST..SYSTIMER_FIRST + SYSTIMER_MASK.count_ones() {
            write(&mut irq, MATRIX + source * u32::from(WORD), LINE);
        }
        write(&mut irq, PRIORITY + LINE * u32::from(WORD), 1);
        write(&mut irq, ENABLE, BIT);
        write(&mut irq, THRESHOLD, 1);
        for target in 0..SYSTIMER_MASK.count_ones() {
            systimer(&mut irq, 1 << target);
            assert_eq!(read(&mut irq, PENDING), BIT);
            assert_eq!(irq.next(), Some(LINE));
            systimer(&mut irq, 0);
            assert_eq!(irq.next(), None);
        }
        write(&mut irq, TYPE, BIT);
        systimer(&mut irq, 1);
        systimer(&mut irq, 2); // Общая линия остаётся высокой.
        systimer(&mut irq, 0);
        let mut snapshot = irq.clone();
        for instance in [&mut irq, &mut snapshot] {
            // Непринятый фронт сохраняется за маской до разрешения.
            write(instance, THRESHOLD, 2);
            assert_eq!(instance.next(), None);
            write(instance, CLEAR, BIT);
            write(instance, CLEAR, 0);
            write(instance, THRESHOLD, 1);
            assert_eq!(instance.next(), Some(LINE));

            instance.claim(LINE);
            assert_eq!(read(instance, PENDING), BIT);
            write(instance, CLEAR, BIT);
            write(instance, CLEAR, 0);
            assert_eq!(instance.next(), None);

            // Отключённый вход позволяет сбросить непринятый фронт.
            systimer(instance, 1);
            systimer(instance, 0);
            assert_eq!(instance.next(), Some(LINE));
            write(instance, ENABLE, 0);
            write(instance, CLEAR, BIT);
            write(instance, CLEAR, 0);
            write(instance, ENABLE, BIT);
            systimer(instance, 0);
            assert_eq!(instance.next(), None);
        }
    }

    #[test]
    fn недоступный_systimer_явно_останавливает_выборку() {
        let mut irq = EspIrq::default();
        write(&mut irq, MATRIX + SYSTIMER_FIRST * u32::from(WORD), LINE);
        write(&mut irq, ENABLE, BIT);
        let fault = BoardFault::UnsupportedRegister {
            pc: PC,
            addr: SYSTIMER_STATUS,
            write: false,
            size: WORD,
            block: "SYSTIMER",
        };
        assert_eq!(irq.sample(PC, None, |_| Err(fault.clone())), Err(fault));
    }

    #[test]
    fn при_равном_приоритете_побеждает_меньший_id() {
        let mut irq = EspIrq::default();
        // Линии стенда и крайний допустимый приоритет по TRM §1.5.2.
        let higher_id = LINE + 1;
        for (i, line) in [higher_id, LINE].into_iter().enumerate() {
            write(&mut irq, MATRIX + (SYSTIMER_FIRST + i as u32) * 4, line);
            write(&mut irq, PRIORITY + line * 4, 15);
        }
        write(&mut irq, THRESHOLD, 15);
        write(&mut irq, ENABLE, BIT | (1 << higher_id));
        systimer(&mut irq, 0b11);
        assert_eq!(irq.next(), Some(LINE));
        write(&mut irq, PRIORITY + LINE * 4, 0);
        assert_eq!(irq.next(), Some(higher_id));
    }
}
