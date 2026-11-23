//! Шина: карта памяти VMCU-C3 и маршрутизация обращений.
//!
//! Подробная карта с регистрами — `crates/sim-mcu/MEMORY_MAP.md`. Адреса флеша и
//! ОЗУ взяты «как у ESP32-C3» (флеш инструкций 0x4200_0000, SRAM 0x3FC8_0000,
//! RTC-память 0x5000_0000, периферия 0x6000_0000), но сами регистры периферии —
//! свои, простые, не ESP32-C3.

use crate::periph::{self, Peripherals};

/// Флеш (код и константы): чтение и исполнение, запись ядром — ошибка доступа.
pub const FLASH_BASE: u32 = 0x4200_0000;
pub const FLASH_SIZE: usize = 4 << 20;
/// ОЗУ: чтение, запись, исполнение.
pub const RAM_BASE: u32 = 0x3fc8_0000;
pub const RAM_SIZE: usize = 384 << 10;
/// Блок конфигурации: хост пишет до сброса, ядро только читает; сброс его не трогает.
pub const CONFIG_BASE: u32 = 0x5000_0000;
pub const CONFIG_SIZE: usize = 4096;
/// CLINT (таймер машинного режима): msip, mtimecmp, mtime — раскладка SiFive CLINT.
pub const CLINT_BASE: u32 = 0x0200_0000;
pub const CLINT_SIZE: u32 = 0x1_0000;
/// Периферия: 0x6000_0000 + 0x1000·n.
pub const PERIPH_BASE: u32 = 0x6000_0000;
pub const PERIPH_SIZE: u32 = 0x8000;

/// Тактовая частота ядра, Гц (как ESP32-C3).
pub const CPU_HZ: u64 = 160_000_000;
/// Частота счёта `mtime`, Гц (как SYSTIMER у ESP32-C3): 1 тик = 10 тактов.
pub const MTIME_HZ: u64 = 16_000_000;
pub const CYCLES_PER_MTIME: u64 = CPU_HZ / MTIME_HZ;

/// Память и периферия.
#[derive(Clone, Debug)]
pub struct Bus {
    pub flash: Vec<u8>,
    /// Массив фиксированного размера — компилятор знает длину и убирает проверки границ.
    pub ram: Box<[u8; RAM_SIZE]>,
    pub config: Vec<u8>,
    pub p: Peripherals,
    /// Было обращение к периферии: ядро должно пересчитать прерывания и события.
    pub dirty: bool,
}

#[inline]
fn read_le(mem: &[u8], off: usize, size: u32) -> u32 {
    match size {
        4 => u32::from_le_bytes([mem[off], mem[off + 1], mem[off + 2], mem[off + 3]]),
        2 => u16::from_le_bytes([mem[off], mem[off + 1]]) as u32,
        _ => mem[off] as u32,
    }
}

impl Bus {
    pub fn new() -> Bus {
        Bus {
            flash: vec![0xff; FLASH_SIZE], // стёртый NOR-флеш читается как 0xFF
            ram: vec![0u8; RAM_SIZE]
                .into_boxed_slice()
                .try_into()
                .expect("размер ОЗУ"),
            config: vec![0xff; CONFIG_SIZE],
            p: Peripherals::default(),
            dirty: false,
        }
    }

    /// Значение `mtime` в момент `cycle`.
    #[inline]
    pub fn mtime(&self, cycle: u64) -> u64 {
        cycle / CYCLES_PER_MTIME
    }

    /// Выборка 16 бит инструкции (флеш или ОЗУ, адрес чётный).
    pub fn fetch16(&self, addr: u32) -> Option<u16> {
        if addr & 1 != 0 {
            return None;
        }
        let f = addr.wrapping_sub(FLASH_BASE) as usize;
        if f < FLASH_SIZE {
            return Some(read_le(&self.flash, f, 2) as u16);
        }
        let r = addr.wrapping_sub(RAM_BASE) as usize;
        if r < RAM_SIZE {
            return Some(read_le(&self.ram[..], r, 2) as u16);
        }
        None
    }

    /// Чтение `size` байт (1, 2, 4) по выровненному адресу. `None` — ошибка доступа.
    pub fn read(&mut self, addr: u32, size: u32, now: u64) -> Option<u32> {
        let r = addr.wrapping_sub(RAM_BASE) as usize;
        if r < RAM_SIZE {
            return Some(read_le(&self.ram[..], r, size));
        }
        let f = addr.wrapping_sub(FLASH_BASE) as usize;
        if f < FLASH_SIZE {
            return Some(read_le(&self.flash, f, size));
        }
        let c = addr.wrapping_sub(CONFIG_BASE) as usize;
        if c < CONFIG_SIZE {
            return Some(read_le(&self.config, c, size));
        }
        let k = addr.wrapping_sub(CLINT_BASE);
        if k < CLINT_SIZE {
            return if size == 4 {
                self.p.clint.read(k, now)
            } else {
                None
            };
        }
        let p = addr.wrapping_sub(PERIPH_BASE);
        if p < PERIPH_SIZE {
            // Снятие байта из FIFO UART меняет линию прерывания.
            if p == (periph::UART0 << 12) | 0x04 {
                self.dirty = true;
            }
            return self.p.read(p, size, now);
        }
        None
    }

    /// Запись `size` байт по выровненному адресу. `false` — ошибка доступа
    /// (флеш, блок конфигурации, несуществующий адрес или регистр).
    pub fn write(&mut self, addr: u32, size: u32, val: u32, now: u64) -> bool {
        let r = addr.wrapping_sub(RAM_BASE) as usize;
        if r < RAM_SIZE {
            let bytes = val.to_le_bytes();
            self.ram[r..r + size as usize].copy_from_slice(&bytes[..size as usize]);
            return true;
        }
        let k = addr.wrapping_sub(CLINT_BASE);
        if k < CLINT_SIZE {
            self.dirty = true;
            return size == 4 && self.p.clint.write(k, val);
        }
        let p = addr.wrapping_sub(PERIPH_BASE);
        if p < PERIPH_SIZE {
            // Пересчёт прерываний/событий нужен только после записей, которые их
            // меняют: SYSCTRL (останов, сброс), INTC, WDT (кроме «кормления» — оно
            // лишь отодвигает событие), RADIO.RX_POP. GPIO/PWM/ADC/UART.TX — нет.
            let (block, reg) = (p >> 12, p & 0xfff);
            if matches!(block, periph::SYSCTRL | periph::INTC)
                || (block == periph::WDT && reg != 0x08)
                || (block == periph::RADIO && reg == 0x10)
            {
                self.dirty = true;
            }
            return self.p.write(p, size, val, now);
        }
        false
    }
}

impl Default for Bus {
    fn default() -> Self {
        Bus::new()
    }
}
