//! Внутренняя память ESP32-C3 и доступные CPU снимки eFuse и strapping.

use std::{mem::size_of, ops::Range, sync::Arc};

use crate::{
    BoardFault,
    memory::{Region, check_data_alignment, read_value},
};

// TRM v1.4 §3.3.2, табл. 3.3-1 и рисунок 3.2-1.
// Ёмкости также приведены в даташите v2.5 §4.1.2.1 «Internal Memory».
const IROM: Range<u32> = 0x4000_0000..0x4006_0000;
const ROM1_IBUS: u32 = 0x4004_0000;
const ROM1_DBUS: Range<u32> = 0x3ff0_0000..0x3ff2_0000;
const IRAM: Range<u32> = 0x4037_c000..0x403e_0000;
const SRAM1_IBUS: u32 = 0x4038_0000;
const SRAM1_DBUS: Range<u32> = 0x3fc8_0000..0x3fce_0000;
const RTC_FAST: Range<u32> = 0x5000_0000..0x5000_2000;

// ESP-IDF components/soc/esp32c3/register/soc/reg_base.h, DR_REG_EFUSE_BASE,
// и ld/esp32c3.peripherals.ld, символ EFUSE. Окно 0x6000_8800 подтверждает
// QEMU esp32c3 (info mtree). В табл. 3.3-3 TRM v1.4 напечатан другой адрес.
// ESP-IDF components/soc/esp32c3/register/soc/efuse_reg.h:
// EFUSE_PGM_DATA0_REG, EFUSE_RD_WR_DIS_REG, EFUSE_RD_SYS_PART2_DATA7_REG,
// EFUSE_RD_REPEAT_DATA0_REG, EFUSE_RD_KEY0_DATA0_REG, EFUSE_RD_KEY1_DATA0_REG,
// EFUSE_CMD_REG, EFUSE_PGM_CMD и EFUSE_DATE_REG (последнее слово).
const EFUSE: Range<u32> = 0x6000_8800..0x6000_8a00;
const EFUSE_DATA_START: u32 = EFUSE.start + 0x2c;
const EFUSE_DATA_END: u32 = EFUSE.start + 0x17c; // Следующее слово: EFUSE_RD_REPEAT_ERR0_REG.
const EFUSE_RD_DIS: u32 = EFUSE.start + 0x30;
const EFUSE_RD_DIS_BITS: usize = 7; // [6:0], TRM §4.3.1.2, блоки 4–10.
const EFUSE_KEY0: u32 = EFUSE.start + 0x9c;
const EFUSE_KEY1: u32 = EFUSE.start + 0xbc;
const EFUSE_CMD: u32 = EFUSE.start + 0x1d4;
const EFUSE_PGM_CMD: u32 = 1 << 1;
const WORD: u8 = size_of::<u32>() as u8;

// TRM §3.3.5.1, табл. 3.3-3 и регистр 5.8 GPIO_STRAP_REG.
const GPIO_STRAP: u32 = 0x6000_4000 + 0x38;

/// Память при доступном SRAM0 (TRM §3.3.2). Содержимое ROM и eFuse
/// задаётся для отдельного экземпляра, уровни strap фиксируются при сбросе хостом.
#[derive(Clone)]
pub struct EspMemory {
    sram: Region,
    rtc: Region,
    rom: Option<Arc<[u8]>>,
    efuse: Option<Arc<[u32]>>,
    straps: Option<u32>,
    legacy_elf: bool,
}

impl Default for EspMemory {
    fn default() -> Self {
        Self {
            sram: Region::new(IRAM.start, (IRAM.end - IRAM.start) as usize, true),
            rtc: Region::new(
                RTC_FAST.start,
                (RTC_FAST.end - RTC_FAST.start) as usize,
                true,
            ),
            rom: None,
            efuse: None,
            straps: None,
            legacy_elf: false,
        }
    }
}

impl EspMemory {
    pub(crate) fn legacy_elf() -> Self {
        // Контракт прежнего холодного запуска ELF: ОЗУ до записи читалось как нули.
        Self {
            legacy_elf: true,
            ..Self::default()
        }
    }

    /// Устанавливает последовательные байты ROM0 и ROM1 по адресам IBUS.
    /// Требуется полный дамп ROM конкретной ревизии.
    pub fn install_rom(&mut self, bytes: &[u8]) -> Result<(), BoardFault> {
        let len = (IROM.end - IROM.start) as usize;
        if bytes.len() != len {
            return Err(BoardFault::Misconfigured(format!(
                "дамп ROM ESP32-C3: нужно {len} байт, передано {}",
                bytes.len()
            )));
        }
        self.rom = Some(bytes.into());
        Ok(())
    }

    /// Устанавливает снимок уже прочитанных CPU слов eFuse, в порядке адресов
    /// от EFUSE_RD_WR_DIS_REG до EFUSE_RD_SYS_PART2_DATA7_REG включительно.
    /// Слова блоков, скрытых RD_DIS, должны быть нулевыми в снимке.
    pub fn install_efuse_readback(&mut self, words: &[u32]) -> Result<(), BoardFault> {
        let len = ((EFUSE_DATA_END - EFUSE_DATA_START) / u32::from(WORD)) as usize;
        if words.len() != len {
            return Err(BoardFault::Misconfigured(format!(
                "снимок eFuse ESP32-C3: нужно {len} слов, передано {}",
                words.len()
            )));
        }
        let rd_dis = words[efuse_index(EFUSE_RD_DIS)];
        let protected = &words[efuse_index(EFUSE_KEY0)..];
        let words_per_block = efuse_index(EFUSE_KEY1) - efuse_index(EFUSE_KEY0);
        for (bit, block) in protected.chunks_exact(words_per_block).enumerate() {
            debug_assert!(bit < EFUSE_RD_DIS_BITS);
            if rd_dis & (1 << bit) != 0 && block.iter().any(|&word| word != 0) {
                return Err(BoardFault::Misconfigured(
                    "снимок eFuse содержит данные из блока, закрытого EFUSE_RD_DIS".into(),
                ));
            }
        }
        self.efuse = Some(words.into());
        Ok(())
    }

    /// Фиксирует фактические уровни на Chip Reset, даташит §3.
    /// Хост задаёт все три уровня, включая плавающие GPIO2 и GPIO8.
    pub fn latch_straps(&mut self, gpio2: bool, gpio8: bool, gpio9: bool) {
        // TRM, регистр 5.8 GPIO_STRAP_REG: GPIO2 → бит 0, GPIO8 → 2, GPIO9 → 3.
        self.straps = Some(u32::from(gpio2) | (u32::from(gpio8) << 2) | (u32::from(gpio9) << 3));
    }

    /// Загружает байты сегмента прошивки в SRAM или RTC FAST.
    /// Алиасы SRAM1 разделяют байты и учитываются при проверке перекрытия.
    pub fn load_ram_segment(&mut self, addr: u32, data: &[u8]) -> Result<(), BoardFault> {
        if data.is_empty() {
            return Err(BoardFault::BadImage("пустой сегмент SRAM ESP32-C3".into()));
        }
        let (region, mapped) = if RTC_FAST.contains(&addr) {
            (&mut self.rtc, addr)
        } else if let Some(mapped) = ram_address(addr, data.len()) {
            (&mut self.sram, mapped)
        } else {
            return Err(BoardFault::BadImage(format!(
                "сегмент по адресу {addr:#010x} вне SRAM ESP32-C3"
            )));
        };
        region.load(mapped, data, addr)
    }

    /// Читает полуслово или слово инструкции RV32IMC (TRM §1.2 и §3.3.2).
    pub fn fetch(&self, pc: u32, size: u8) -> Result<u32, BoardFault> {
        if !matches!(size, 2 | WORD) || !pc.is_multiple_of(2) {
            return Err(BoardFault::Unaligned { pc, addr: pc, size });
        }
        if IROM.contains(&pc) {
            return self.read_rom(pc, pc, pc, size);
        }
        let region = if IRAM.contains(&pc) {
            &self.sram
        } else {
            &self.rtc
        };
        region
            .read(pc, size, true)
            .ok_or_else(|| bus_error(pc, pc, false))
    }

    /// Читает данные по карте внутренней памяти.
    pub fn read(&mut self, pc: u32, addr: u32, size: u8) -> Result<u32, BoardFault> {
        check_data_alignment(pc, addr, size)?;
        self.access(pc, addr, size, None)?
            .ok_or_else(|| bus_error(pc, addr, false))
    }

    /// Записывает данные в SRAM или RTC FAST.
    pub fn write(&mut self, pc: u32, addr: u32, size: u8, value: u32) -> Result<(), BoardFault> {
        check_data_alignment(pc, addr, size)?;
        self.access(pc, addr, size, Some(value))?
            .map(|_| ())
            .ok_or_else(|| bus_error(pc, addr, true))
    }

    pub(crate) fn access(
        &mut self,
        pc: u32,
        addr: u32,
        size: u8,
        value: Option<u32>,
    ) -> Result<Option<u32>, BoardFault> {
        let write = value.is_some();
        if (IROM.contains(&addr) || IRAM.contains(&addr)) && size != WORD {
            // TRM §3.3.1: обращения к данным через IBUS выполняются словами.
            return Err(bus_error(pc, addr, write));
        }
        if IROM.contains(&addr) || ROM1_DBUS.contains(&addr) {
            if write {
                return Err(BoardFault::WriteToReadOnly { pc, addr });
            }
            let mapped = if ROM1_DBUS.contains(&addr) {
                ROM1_IBUS + (addr - ROM1_DBUS.start)
            } else {
                addr
            };
            return self.read_rom(pc, addr, mapped, size).map(Some);
        }
        if addr == GPIO_STRAP || EFUSE.contains(&addr) {
            return self.register(pc, addr, size, value).map(Some);
        }
        let (region, mapped) = if RTC_FAST.contains(&addr) {
            (&mut self.rtc, addr)
        } else if let Some(mapped) = ram_address(addr, size.into()) {
            (&mut self.sram, mapped)
        } else {
            return Ok(None);
        };
        if let Some(value) = value {
            return region
                .write(mapped, size, value)
                .map(|()| Some(0))
                .ok_or_else(|| bus_error(pc, addr, true));
        }
        region
            .read(mapped, size, !self.legacy_elf)
            .map(Some)
            .ok_or_else(|| {
                BoardFault::Unsupported(format!("содержимое SRAM по адресу {addr:#010x} не задано"))
            })
    }

    fn read_rom(&self, pc: u32, addr: u32, mapped: u32, size: u8) -> Result<u32, BoardFault> {
        if u32::from(size) > IROM.end - mapped {
            return Err(bus_error(pc, addr, false));
        }
        let rom = self.rom.as_ref().ok_or_else(|| {
            BoardFault::Unsupported(format!(
                "дамп ROM ESP32-C3 для адреса {addr:#010x} не задан"
            ))
        })?;
        Ok(read_value(&rom[(mapped - IROM.start) as usize..], size))
    }

    fn register(
        &self,
        pc: u32,
        addr: u32,
        size: u8,
        value: Option<u32>,
    ) -> Result<u32, BoardFault> {
        let write = value.is_some();
        let unsupported = || BoardFault::UnsupportedRegister {
            pc,
            addr,
            write,
            size,
            block: if addr == GPIO_STRAP {
                "GPIO_STRAP"
            } else {
                "eFuse"
            },
        };
        if size != WORD {
            return Err(unsupported());
        }
        if addr == GPIO_STRAP {
            if write {
                return Err(BoardFault::WriteToReadOnly { pc, addr });
            }
            return self.straps.ok_or_else(|| {
                BoardFault::Unsupported("уровни GPIO2, GPIO8, GPIO9 при сбросе не заданы".into())
            });
        }
        if (EFUSE_DATA_START..EFUSE_DATA_END).contains(&addr) {
            if write {
                return Err(BoardFault::WriteToReadOnly { pc, addr });
            }
            return self
                .efuse
                .as_ref()
                .map(|words| words[efuse_index(addr)])
                .ok_or_else(|| {
                    BoardFault::Unsupported(format!(
                        "снимок eFuse для адреса {addr:#010x} не задан"
                    ))
                });
        }
        // TRM §4.3.2: защита снимка также блокирует буфер прожига и команду PGM_CMD.
        if let Some(value) = value
            && (addr < EFUSE_DATA_START || (addr == EFUSE_CMD && value & EFUSE_PGM_CMD != 0))
        {
            return Err(BoardFault::WriteToReadOnly { pc, addr });
        }
        Err(unsupported())
    }
}

fn ram_address(addr: u32, len: usize) -> Option<u32> {
    // TRM §3.3.2: окна SRAM1 идут в одинаковом порядке.
    if SRAM1_DBUS.contains(&addr) && len <= (SRAM1_DBUS.end - addr) as usize {
        Some(SRAM1_IBUS + (addr - SRAM1_DBUS.start))
    } else if IRAM.contains(&addr) && len <= (IRAM.end - addr) as usize {
        Some(addr)
    } else {
        None
    }
}

fn efuse_index(addr: u32) -> usize {
    ((addr - EFUSE_DATA_START) / u32::from(WORD)) as usize
}

fn bus_error(pc: u32, addr: u32, write: bool) -> BoardFault {
    BoardFault::BusError { pc, addr, write }
}
