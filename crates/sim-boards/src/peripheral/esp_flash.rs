//! Функциональная модель чтения flash ESP32-C3. Такты SPI, вытеснение
//! строк кэша, шифрование и запись flash требуют отдельных моделей.

use std::{
    cell::{Cell, RefCell},
    collections::BTreeMap,
    ops::Range,
    sync::Arc,
};

use crate::esp_reset_profile::EspResetValues;
use crate::{
    BoardFault,
    esp_image::{EspImage, FlashImage, PartitionTable, SegmentKind},
};

// TRM v1.4 §3.3.3.1–2, таблица 3.3-2; ESP-IDF v6.1.1
// soc/esp32c3/include/soc/ext_mem_defs.h и hal/esp32c3/include/hal/mmu_ll.h.
const PAGE: u32 = 64 * 1024;
const LINE: u32 = 32;
const ENTRIES: usize = 128;
const PHYSICAL_LIMIT: u32 = 256 * PAGE;
const PAGE_NUMBER: u32 = 0xff;
const INVALID: u32 = 1 << 8;
const WINDOW_LEN: u32 = ENTRIES as u32 * PAGE;
const DROM: Range<u32> = 0x3c00_0000..0x3c00_0000 + WINDOW_LEN;
const IROM: Range<u32> = 0x4200_0000..0x4200_0000 + WINDOW_LEN;
const WORD: u8 = size_of::<u32>() as u8;

// TRM §3.3.5.1, табл. 3.3-3; soc/esp32c3/register/soc/reg_base.h.
const SPI1: u32 = 0x6000_2000;
const SPI0: u32 = 0x6000_3000;
const SPI_END: u32 = 0x6000_4000;
const EXTMEM: u32 = 0x600c_4000;
const MMU: u32 = 0x600c_5000;
const MMU_END: u32 = MMU + ENTRIES as u32 * WORD as u32;

// soc/esp32c3/register/soc/extmem_reg.h: EXTMEM_ICACHE_* и CACHE_STATE.
const CACHE_CTRL: u32 = EXTMEM;
const CACHE_ENABLE: u32 = 1 << 0;
const CACHE_BUS: u32 = EXTMEM + 0x004;
const TAG_POWER: u32 = EXTMEM + 0x008;
const MMU_POWER: u32 = EXTMEM + 0x0ac;
const CACHE_MEM_FORCE_PU: u32 = 1 << 2;
const CACHE_MEM_FORCE_ON: u32 = 1;
const SHUT_IBUS: u32 = 1 << 0;
const SHUT_DBUS: u32 = 1 << 1;
const SYNC_CTRL: u32 = EXTMEM + 0x028;
const INVALIDATE: u32 = 1 << 0;
const SYNC_DONE: u32 = 1 << 1;
const SYNC_ADDR: u32 = EXTMEM + 0x02c;
const SYNC_SIZE: u32 = EXTMEM + 0x030;
const SYNC_SIZE_MASK: u32 = 0x007f_ffff;
const PRELOAD_CTRL: u32 = EXTMEM + 0x034;
const PRELOAD_DONE: u32 = 1 << 1; // EXTMEM_ICACHE_PRELOAD_DONE, сброс 1.
const AUTOLOAD_CTRL: u32 = EXTMEM + 0x040;
const AUTOLOAD_DONE: u32 = 1 << 3; // EXTMEM_ICACHE_AUTOLOAD_DONE, сброс 1.
const CACHE_STATE: u32 = EXTMEM + 0x0b0;
const CACHE_IDLE: u32 = 1;
const CACHE_ILG_ENABLE: u32 = EXTMEM + 0x078;
const CACHE_ILG_CLEAR: u32 = EXTMEM + 0x07c;
const CACHE_ILG_STATUS: u32 = EXTMEM + 0x080;
const CACHE_ILG_W1C: u32 = (0b11 << 7) | (1 << 5) | 0b11;
const MMU_FAULT: u32 = 1 << 5; // EXTMEM_MMU_ENTRY_FAULT_ST.
const SYNC_FAULT: u32 = 1 << 0; // EXTMEM_ICACHE_SYNC_OP_FAULT_ST.
const CACHE_CORE_ENABLE: u32 = EXTMEM + 0x084;
const CACHE_CORE_CLEAR: u32 = EXTMEM + 0x088;
const CACHE_CORE_STATUS: u32 = EXTMEM + 0x08c;
const CACHE_CORE_MASK: u32 = 0x3f;
const IBUS_MASKED: u32 = 1;
const DBUS_MASKED: u32 = 1 << 3;
const IBUS_WRITE: u32 = 1 << 1;
const DBUS_WRITE: u32 = 1 << 5;
const CACHE_WRAP: u32 = EXTMEM + 0x0a8;
const MMU_OWNER: u32 = EXTMEM + 0x0c4;
const MMU_BOTH_BUSES: u32 = 0xf; // EXTMEM_CACHE_MMU_OWNER: [2,0] IBUS, [3,1] DBUS.
const CACHE_FREEZE: u32 = EXTMEM + 0x0cc;
const FREEZE_ENABLE: u32 = 1 << 0;
const FREEZE_DONE: u32 = 1 << 2;

// TRM рег. 16.7, ESP-IDF soc/esp32c3/register/soc/system_reg.h.
const SYSTEM_CACHE_CONTROL: u32 = 0x600c_0040;
const ICACHE_CLOCK: u32 = 1 << 0;
const ICACHE_RESET: u32 = 1 << 1;
const DCACHE_CLOCK: u32 = 1 << 2;

// TRM §3.3.2 и §14.4.2.1, регистры 14.5/14.6; sensitive_reg.h.
const SRAM0: Range<u32> = 0x4037_c000..0x4038_0000;
const SRAM_LOCK: u32 = 0x600c_1018;
const SRAM_USAGE: u32 = 0x600c_101c;
const SRAM0_CPU: u32 = 1 << 0;
const SRAM_USAGE_RESERVED: u32 = 0b111 << 1; // TRM регистр 14.6, reset [3:1].
const LOCK: u32 = 1;

// soc/esp32c3/register/soc/spi_mem_reg.h, SPI_MEM_*.
const CMD: u32 = 0x000;
const ADDR: u32 = 0x004;
const CTRL: u32 = 0x008;
const CTRL1: u32 = 0x00c;
const CTRL2: u32 = 0x010;
const CLOCK: u32 = 0x014;
const USER: u32 = 0x018;
const USER1: u32 = 0x01c;
const USER2: u32 = 0x020;
const MOSI_DLEN: u32 = 0x024;
const MISO_DLEN: u32 = 0x028;
const RD_STATUS: u32 = 0x02c;
const MISC: u32 = 0x034;
const CACHE_FCTRL: u32 = 0x03c;
const FSM: u32 = 0x054;
const W0: u32 = 0x058;
const W15: u32 = 0x094;
const WAIT_IDLE_CTRL: u32 = 0x098; // FLASH_WAITI_CTRL, [15:1].
const WAIT_IDLE_MASK: u32 = 0xfffe;
const WAIT_IDLE_RESET: u32 = 0x05 << 2; // WAITI_CMD=RDSR, dummy выключен.
const SUS_CTRL: u32 = 0x09c;
const SUS_CTRL_RESET: u32 = (4 << 25) | (0x80 << 6); // SUS_TIMEOUT_CNT, PESR_END_MSK.
const SUS_CTRL_ACTIVE: u32 = (1 << 5) | 0b11; // FLASH_PES_EN, FLASH_PES и FLASH_PER.
const SUS_STATUS: u32 = 0x0a4;
const CLOCK_GATE: u32 = 0x0dc;
const CORE_CLK_SEL: u32 = 0x0e0; // SPI_MEM_SPI01_CLK_SEL[1:0], reset 0, 3 зарезервировано.
const CLOCK_FORCE_ON: u32 = 1; // SPI_MEM_CLK_EN, reset 1.
const SPI_BYTES: usize = (W15 - W0 + WORD as u32) as usize;
const FLASH_READ: u32 = 1 << 31;
const FLASH_RDID: u32 = 1 << 28;
const FLASH_RDSR: u32 = 1 << 27;
const FLASH_RES: u32 = 1 << 20;
const RESANDRES: u32 = 1 << 15;
const USR: u32 = 1 << 18;
const USR_COMMAND: u32 = 1 << 31;
const USR_ADDR: u32 = 1 << 30;
const USR_DUMMY: u32 = 1 << 29;
const USR_MISO: u32 = 1 << 28;
const USR_MISO_HIGHPART: u32 = 1 << 24;
const CS_TIMING: u32 = (1 << 7) | (1 << 6); // SPI_MEM_CS_SETUP, SPI_MEM_CS_HOLD.
// CTRL2: CS_HOLD_DELAY[30:25], CS_HOLD_TIME[9:5], CS_SETUP_TIME[4:0].
const CS_TIME_MASK: u32 = (0x3f << 25) | 0x3ff;
const CS_TIME_RESET: u32 = (1 << 5) | 1;
const SYNC_RESET: u32 = 1 << 31; // SPI_MEM_SYNC_RESET, сброс FSM.
const ADDR_SHIFT: u32 = 26;
const ADDR_LEN: u32 = 0x3f;
const DUMMY_LEN: u32 = 0x3f;
const COMMAND_SHIFT: u32 = 28;
const COMMAND_VALUE: u32 = 0xffff;
const BIT_LEN: u32 = 0x3ff;
const DATA_STATUS: u32 = 0x00ff_ffff; // STATUS[15:0], WB_MODE[23:16].
const CS0_DIS: u32 = 1 << 0;
const CS1_DIS: u32 = 1 << 1;
const CK_IDLE_EDGE: u32 = 1 << 9;
const CACHE_REQ_EN: u32 = 1 << 0;
const CACHE_USR_ADDR_4B: u32 = 1 << 1;
const CACHE_USR_CMD: u32 = 1 << 2;
const FREAD_DIO: u32 = 1 << 23;
const FREAD_DUAL: u32 = 1 << 14;
const FASTRD: u32 = 1 << 13;
const FDUMMY_OUT: u32 = 1 << 3;
// SPI_MEM_WP_REG, D_POL, Q_POL, RESANDRES, FASTRD_MODE: значения сброса.
const CTRL_RESET: u32 = (1 << 21) | (1 << 19) | (1 << 18) | RESANDRES | FASTRD;
const CTRL1_RW: u32 = 0xfff; // CS_HOLD_DLY_RES[11:2], CLK_MODE[1:0].
const CTRL1_RESET: u32 = 0x3ff << 2;
const CLOCK_MASK: u32 = 0x00ff_ffff; // SPI_MEM_CLKCNT_N/H/L.
const CLOCK_RESET: u32 = 0x0003_0103; // N=3, H=1, L=3.
// SPI_MEM_CLK_EQU_SYSCLK. spi_mem_reg.h называет бит зарезервированным
// у SPI1, но esp_rom_spiflash_config_clk(freqdiv, spi) из ROM rev. 3
// (0x4004dd4a) явно пишет его в SPI0 и SPI1 при freqdiv=1.
// Модель сохраняет настройку, задержки SCLK на выводе не рассчитывает.
const CLOCK_SYSCLK: u32 = 1 << 31;
// SPI_MEM_FSM: CSPI_LOCK_DELAY_TIME[11:7], EM_ST/CSPI_ST=idle между транзакциями.
const LOCK_DELAY_MASK: u32 = 0x1f << 7;
const LOCK_DELAY_RESET: u32 = 4 << 7;

// spi_flash/include/esp_flash_chips/spi_flash_defs.h.
const READ: u32 = 0x03;
const READ_4B: u32 = 0x13;
const FAST_READ: u32 = 0x0b;
const FAST_READ_4B: u32 = 0x0c;
const DUAL_OUT: u32 = 0x3b;
const DUAL_OUT_4B: u32 = 0x3c;
const DUAL_IO: u32 = 0xbb;
const DUAL_IO_4B: u32 = 0xbc;
const RDID: u32 = 0x9f;
const RDSFDP: u32 = 0x5a;
const RDSR: u32 = 0x05;
const RDSR2: u32 = 0x35;
const RESUME: u32 = 0x7a;
const WRDI: u32 = 0x04;
const STATUS_BUSY_OR_WREN: u8 = (1 << 0) | (1 << 1); // SR_WIP, SR_WREN.
const RDSR2_FIELD: &str = "RDSR2";

pub(super) fn supports_profile_byte(address: u32, field: &str, value: u32) -> bool {
    address == SPI1 + W0
        && u8::try_from(value).is_ok()
        && (field == RDSR2_FIELD
            || field
                .strip_prefix("SFDP.0x")
                .and_then(|offset| u32::from_str_radix(offset, 16).ok())
                .is_some_and(|offset| {
                    offset < PHYSICAL_LIMIT && field == format!("SFDP.{offset:#x}")
                }))
}

/// Ответы конкретной микросхемы flash на RDID и RDSR. Порядок JEDEC-байтов
/// соответствует порядку на шине. Второй регистр статуса зависит от модели.
#[derive(Clone, Copy)]
pub struct EspFlashReadback {
    pub jedec_id: [u8; 3],
    pub status1: u8,
    pub status2: Option<u8>,
}

/// Состояние SPI0/1, общей MMU, кэша и принадлежащего ему банка SRAM0.
/// Клон разделяет только неизменяемые исходные байты и таблицу разделов.
#[derive(Clone)]
pub struct EspFlash {
    bytes: Arc<[u8]>,
    capacity: u32,
    partitions: Option<Arc<PartitionTable>>,
    readback: Option<EspFlashReadback>,
    spi0: SpiMem,
    spi1: SpiMem,
    cache: Cache,
    spi_clock: bool,
    spi_reset: bool,
}

impl EspFlash {
    /// Физический снимок от нулевого смещения. Отсутствующий хвост снимка
    /// неизвестен. Для стёртого участка передаются его байты.
    pub fn new(bytes: &[u8], capacity: u32) -> Result<Self, BoardFault> {
        if capacity == 0 || capacity > PHYSICAL_LIMIT || bytes.len() > capacity as usize {
            return Err(BoardFault::BadImage(format!(
                "длина снимка flash {} несовместима с ёмкостью {capacity:#x}",
                bytes.len()
            )));
        }
        Ok(Self {
            bytes: bytes.into(),
            capacity,
            partitions: None,
            readback: None,
            spi0: SpiMem::default(),
            spi1: SpiMem::default(),
            cache: Cache::default(),
            // ESP-IDF system_reg.h: SPI01_CLK_EN=1, SPI01_RST=0 после сброса.
            spi_clock: true,
            spi_reset: false,
        })
    }

    /// Принимает проверенный образ из 2.1 и сохраняет его таблицу разделов.
    /// Ёмкость берётся из заголовка загрузчика. Исходные байты должны быть открытыми.
    pub fn from_image(image: &FlashImage<'_>) -> Result<Self, BoardFault> {
        let mut flash = Self::new(image.as_bytes(), image.bootloader().header.flash_size)?;
        flash.partitions = Some(Arc::new(image.partition_table().clone()));
        Ok(flash)
    }

    /// Устанавливает паспортный или измеренный ответ для неподвижного снимка.
    pub fn set_readback(&mut self, readback: EspFlashReadback) -> Result<(), BoardFault> {
        if readback.status1 & STATUS_BUSY_OR_WREN != 0 {
            return Err(BoardFault::Misconfigured(
                "снимок flash содержит установленный WIP или WEL".into(),
            ));
        }
        self.readback = Some(readback);
        Ok(())
    }

    /// Холодный сброс контроллеров. Содержимое flash и её профиль сохраняются.
    pub fn reset(&mut self) {
        self.cache = Cache::default();
        self.spi0 = SpiMem::default();
        self.spi1 = SpiMem::default();
        self.spi_clock = true;
        self.spi_reset = false;
    }

    pub(crate) fn set_spi01_control(&mut self, clock: bool, reset: bool) {
        // system_reg.h, SYSTEM_SPI01_{CLK_EN,RST}. EXTMEM и MMU имеют свои регистры.
        self.spi_clock = clock;
        if reset && !self.spi_reset {
            self.spi0 = SpiMem::default();
            self.spi1 = SpiMem::default();
        }
        self.spi_reset = reset;
    }

    /// Читает физические байты, как путь без кэша для ROM.
    pub fn read_physical(&self, offset: u32, len: usize) -> Result<&[u8], BoardFault> {
        let span = range(offset, len, self.capacity).ok_or_else(|| {
            BoardFault::BadImage(format!("чтение flash {offset:#x}+{len:#x} вне ёмкости"))
        })?;
        if span.is_empty() {
            return Ok(&[]);
        }
        self.bytes.get(span).ok_or_else(|| {
            BoardFault::Unsupported(format!(
                "снимок flash не содержит байты {offset:#x}+{len:#x}"
            ))
        })
    }

    /// Читает открытый раздел по метке. Формула границ из
    /// components/esp_partition/partition_target.c, esp_partition_read.
    pub fn read_partition(
        &self,
        label: &str,
        offset: u32,
        len: usize,
    ) -> Result<&[u8], BoardFault> {
        let table = self
            .partitions
            .as_ref()
            .ok_or_else(|| BoardFault::Unsupported("таблица разделов flash не загружена".into()))?;
        let partition = table
            .partitions
            .iter()
            .find(|partition| partition.label == label)
            .ok_or_else(|| BoardFault::BadImage(format!("нет раздела {label:?}")))?;
        range(offset, len, partition.size).ok_or_else(|| {
            BoardFault::BadImage(format!("чтение {offset:#x}+{len:#x} вне раздела {label:?}"))
        })?;
        if partition.encrypted {
            return Err(BoardFault::Unsupported(format!(
                "для раздела {label:?} с флагом шифрования нужен профиль шифрования"
            )));
        }
        self.read_physical(partition.offset + offset, len)
    }

    /// Настраивает записи для DROM/IROM из разобранного образа, сохраняя
    /// остальную MMU. Включение кэша и снятие масок шин выполняются отдельно.
    /// ESP-IDF bootloader_support/src/bootloader_utility.c, set_cache_and_start_app.
    pub fn map_image(&mut self, image: &EspImage<'_>) -> Result<(), BoardFault> {
        let mut mapping = [None; ENTRIES];
        for segment in &image.segments {
            let window = match segment.kind {
                SegmentKind::Drom => &DROM,
                SegmentKind::Irom => &IROM,
                _ => continue,
            };
            let start = segment.load_addr.wrapping_sub(window.start);
            let len = segment.data.len();
            let span = range(start, len, WINDOW_LEN)
                .filter(|r| !r.is_empty())
                .ok_or_else(|| BoardFault::BadImage("сегмент вне окна DROM/IROM".into()))?;
            if start % PAGE != segment.flash_offset % PAGE {
                return Err(BoardFault::BadImage(
                    "смещения сегмента внутри физической и виртуальной страниц различаются".into(),
                ));
            }
            if self.read_physical(segment.flash_offset, len)? != segment.data {
                return Err(BoardFault::BadImage(
                    "байты сегмента отличаются от flash".into(),
                ));
            }
            let first = span.start / PAGE as usize;
            let last = (span.end - 1) / PAGE as usize;
            let physical = segment.flash_offset / PAGE;
            for (delta, slot) in mapping[first..=last].iter_mut().enumerate() {
                let entry = physical + delta as u32;
                if slot.is_some_and(|previous| previous != entry) {
                    return Err(BoardFault::BadImage(format!(
                        "сегменты требуют разных физических страниц для записи MMU {}",
                        first + delta
                    )));
                }
                *slot = Some(entry);
            }
        }
        for (entry, value) in self.cache.mmu.iter_mut().zip(mapping) {
            if let Some(value) = value {
                *entry = value;
            }
        }
        Ok(())
    }

    /// Доступ к данным и MMIO. Ok(None) передаёт адрес остальной шине.
    /// SRAM0 обслуживается этим методом до обращения к общей карте SRAM.
    pub fn access(
        &mut self,
        pc: u32,
        addr: u32,
        size: u8,
        value: Option<u32>,
    ) -> Result<Option<u32>, BoardFault> {
        self.access_with_profile(pc, addr, size, value, &mut EspResetValues::default())
    }

    pub(crate) fn access_with_profile(
        &mut self,
        pc: u32,
        addr: u32,
        size: u8,
        value: Option<u32>,
        profile: &mut EspResetValues,
    ) -> Result<Option<u32>, BoardFault> {
        let write = value.is_some();
        if let Some((linear, bus)) = cache_address(addr) {
            check_alignment(pc, addr, size)?;
            if write {
                self.cache.report_access(bus, IBUS_WRITE, DBUS_WRITE);
                return Err(BoardFault::WriteToReadOnly { pc, addr });
            }
            if bus == SHUT_IBUS && size != WORD {
                return Err(bus_error(pc, addr, write)); // TRM §3.3.1: данные через IBUS словами.
            }
            return self.read_cached(pc, addr, linear, bus, size).map(Some);
        }
        if SRAM0.contains(&addr) {
            check_alignment(pc, addr, size)?;
            if size != WORD || !self.cache.sram0_cpu {
                return Err(bus_error(pc, addr, write));
            }
            let word = &mut self.cache.sram0[((addr - SRAM0.start) / u32::from(WORD)) as usize];
            if let Some(value) = value {
                *word = Some(value);
                return Ok(Some(0));
            }
            return word.map(Some).ok_or_else(|| unknown_sram(addr));
        }

        let block = if (SPI1..SPI0).contains(&addr) {
            "SPI1 flash"
        } else if (SPI0..SPI_END).contains(&addr) {
            "SPI0 flash"
        } else if (EXTMEM..MMU_END).contains(&addr)
            || matches!(addr, SRAM_LOCK | SRAM_USAGE | SYSTEM_CACHE_CONTROL)
        {
            "cache/MMU flash"
        } else {
            return Ok(None);
        };
        check_alignment(pc, addr, size)?;
        let unknown = BoardFault::UnsupportedRegister {
            pc,
            addr,
            size,
            write,
            block,
        };
        if size != WORD {
            return Err(unknown);
        }

        if (SPI1..SPI_END).contains(&addr) && (!self.spi_clock || self.spi_reset) {
            return Err(unknown);
        }
        if (SPI1..SPI0).contains(&addr) {
            let offset = addr - SPI1;
            if let (CMD, Some(command)) = (offset, value) {
                self.spi_command(command, profile)?;
                return Ok(Some(0));
            }
            return self.spi1.register(offset, value, false, unknown).map(Some);
        }
        if (SPI0..SPI_END).contains(&addr) {
            return self
                .spi0
                .register(addr - SPI0, value, true, unknown)
                .map(Some);
        }
        self.cache.register(pc, addr, value, unknown).map(Some)
    }

    /// Выборка инструкции через IROM или SRAM0. Как и шина RV32IMC,
    /// вызывающий может читать отдельные полуслова по обе стороны границы.
    pub fn fetch(&self, pc: u32, size: u8) -> Result<Option<u32>, BoardFault> {
        if DROM.contains(&pc) {
            return Err(bus_error(pc, pc, false));
        }
        if !IROM.contains(&pc) && !SRAM0.contains(&pc) {
            return Ok(None);
        }
        if !matches!(size, 2 | WORD) || !pc.is_multiple_of(2) {
            return Err(BoardFault::Unaligned { pc, addr: pc, size });
        }
        if IROM.contains(&pc) {
            return self
                .read_cached(pc, pc, pc - IROM.start, SHUT_IBUS, size)
                .map(Some);
        }
        if !self.cache.sram0_cpu || u32::from(size) > SRAM0.end - pc {
            return Err(bus_error(pc, pc, false));
        }
        let offset = pc - SRAM0.start;
        let index = (offset / u32::from(WORD)) as usize;
        let low = self.cache.sram0[index].ok_or_else(|| unknown_sram(pc))?;
        let shift = offset % u32::from(WORD) * u8::BITS;
        let result = match (size, shift) {
            (2, _) => (low >> shift) & u32::from(u16::MAX),
            (WORD, 0) => low,
            _ => {
                let high = self.cache.sram0[index + 1].ok_or_else(|| unknown_sram(pc + 2))?;
                (low >> shift) | (high << (u32::BITS - shift))
            }
        };
        Ok(Some(result))
    }

    fn read_cached(
        &self,
        pc: u32,
        addr: u32,
        linear: u32,
        bus: u32,
        size: u8,
    ) -> Result<u32, BoardFault> {
        if !self.spi_clock || self.spi_reset {
            return Err(BoardFault::Unsupported(
                "доступ к flash через кэш при закрытых тактах или сбросе SPI0/1".into(),
            ));
        }
        if !self.cache.enabled
            || self.cache.frozen
            || self.cache.clock_control & (ICACHE_CLOCK | ICACHE_RESET) != ICACHE_CLOCK
            || self.cache.shut & bus != 0
        {
            self.cache.report_access(bus, IBUS_MASKED, DBUS_MASKED);
            return Err(bus_error(pc, addr, false));
        }
        if self.cache.sram0_cpu
            || self.spi0.cache_fctrl & CACHE_REQ_EN == 0
            || u32::from(size) > WINDOW_LEN - linear
        {
            return Err(bus_error(pc, addr, false));
        }
        if self.spi0.misc & CS0_DIS != 0 || self.spi0.misc & CS1_DIS == 0 {
            return Err(BoardFault::Unsupported(
                "SPI0: flash должна быть выбрана на CS0".into(),
            ));
        }
        if self.spi0.read_mode() == (FREAD_DIO | FREAD_DUAL) {
            return Err(BoardFault::Misconfigured(
                "SPI0: одновременно выбраны DIO и DOUT".into(),
            ));
        }
        if self.spi0.cache_fctrl & CACHE_USR_CMD != 0 {
            let addr_bits = self.spi0.read_address_bits()?;
            let configured_bits = if self.spi0.cache_fctrl & CACHE_USR_ADDR_4B == 0 {
                24
            } else {
                32
            };
            if addr_bits != configured_bits {
                return Err(BoardFault::Misconfigured(
                    "SPI0: длина адреса кэша расходится с USER1".into(),
                ));
            }
        }

        let mut bytes = [0; WORD as usize];
        let mut lines = self.cache.lines.borrow_mut();
        let last = linear + u32::from(size) - 1;
        for (delta, byte) in bytes[..usize::from(size)].iter_mut().enumerate() {
            let at = linear + delta as u32;
            let entry = self.cache.mmu[(at / PAGE) as usize];
            if entry & INVALID != 0 {
                self.cache.illegal.set(self.cache.illegal.get() | MMU_FAULT);
                return Err(bus_error(pc, addr, false));
            }
            if lines.get(&(at / LINE)).is_some_and(|&old| old != entry) {
                return Err(BoardFault::Unsupported(format!(
                    "отображение строки кэша по {addr:#010x} изменилось без инвалидации"
                )));
            }
            *byte = self.read_physical(entry * PAGE + at % PAGE, 1)?[0];
        }
        for line in linear / LINE..=last / LINE {
            lines.insert(line, self.cache.mmu[(line * LINE / PAGE) as usize]);
        }
        Ok(u32::from_le_bytes(bytes))
    }

    pub(crate) fn interrupt_levels(&self) -> (bool, bool) {
        (
            self.cache.illegal.get() & self.cache.illegal_enable != 0,
            self.cache.core_access.get() & self.cache.core_enable != 0,
        )
    }

    fn spi_command(
        &mut self,
        command: u32,
        profile: &mut EspResetValues,
    ) -> Result<(), BoardFault> {
        if command == 0 {
            return Ok(());
        }
        if self.spi1.misc & CS0_DIS != 0 || self.spi1.misc & CS1_DIS == 0 {
            return Err(BoardFault::Unsupported(
                "SPI1: flash должна быть выбрана на CS0".into(),
            ));
        }
        // FLASH_RES без RESANDRES только выводит flash из power-down.
        // Вход в power-down отклоняется, поэтому снимок уже активен.
        if command == FLASH_RES && self.spi1.ctrl & RESANDRES == 0 {
            return Ok(());
        }
        let mut reply = [0; SPI_BYTES];
        let (start, len) = match command {
            FLASH_READ => {
                let len = self.spi1.rx_len(SPI_BYTES)?;
                let addr = self.spi1.physical_address(self.spi1.address_bits())?;
                reply[..len].copy_from_slice(self.read_physical(addr, len)?);
                (0, len)
            }
            FLASH_RDID => {
                let id = self.readback()?.jedec_id;
                reply[..id.len()].copy_from_slice(&id);
                (0, id.len())
            }
            FLASH_RDSR => {
                let status = self.readback()?.status1;
                self.spi1.status = self.spi1.status & !u32::from(u8::MAX) | u32::from(status);
                return Ok(());
            }
            USR => {
                let spi = &self.spi1;
                let opcode = spi.opcode()?;
                let phases = spi.user & !(USR_MISO_HIGHPART | CS_TIMING);
                if matches!(opcode, RESUME | WRDI) {
                    if phases != USR_COMMAND {
                        return Err(BoardFault::Misconfigured(
                            "SPI1: RESUME/WRDI требуют только фазу команды".into(),
                        ));
                    }
                    // spi_flash_defs.h и bootloader_flash_config_esp32c3.c:
                    // возобновлять нечего, WEL уже сброшен в снимке только для чтения.
                    return Ok(());
                }
                let start = if spi.user & USR_MISO_HIGHPART != 0 {
                    SPI_BYTES / 2
                } else {
                    0
                };
                let len = spi.rx_len(SPI_BYTES - start)?;
                if matches!(opcode, RDID | RDSR | RDSR2) {
                    if phases != USR_COMMAND | USR_MISO {
                        return Err(BoardFault::Misconfigured(
                            "SPI1: неверные фазы RDID/RDSR".into(),
                        ));
                    }
                    let readback = self.readback()?;
                    let status2;
                    let reply_bytes: &[u8] = match opcode {
                        RDID => &readback.jedec_id,
                        RDSR => std::slice::from_ref(&readback.status1),
                        _ => {
                            if len != size_of::<u8>() {
                                return Err(BoardFault::Unsupported(
                                    "SPI1: требуется однобайтовый ответ RDSR2".into(),
                                ));
                            }
                            let word = match readback.status2 {
                                Some(value) => Some(u32::from(value)),
                                None => profile.read(SPI1 + W0, RDSR2_FIELD)?,
                            };
                            status2 = word.ok_or_else(|| {
                                BoardFault::Unsupported("не задан ответ flash на RDSR2".into())
                            })? as u8;
                            std::slice::from_ref(&status2)
                        }
                    };
                    reply[..len].copy_from_slice(reply_bytes.get(..len).ok_or_else(|| {
                        BoardFault::Unsupported("SPI1: длина ответа RDID/RDSR вне профиля".into())
                    })?);
                } else {
                    let bits = spi.read_address_bits()?;
                    let addr = spi.physical_address(bits)?;
                    if opcode == RDSFDP {
                        // SFDP — отдельное адресное пространство микросхемы,
                        // его байтов нет в дампе основного массива flash.
                        let mut next = profile.clone();
                        for (index, byte) in reply[..len].iter_mut().enumerate() {
                            let offset = addr + index as u32;
                            let field = format!("SFDP.{offset:#x}");
                            *byte = next.read(SPI1 + W0, &field)?.ok_or_else(|| {
                                BoardFault::Unsupported(format!("не задан байт flash {field}"))
                            })? as u8;
                        }
                        *profile = next;
                    } else {
                        reply[..len].copy_from_slice(self.read_physical(addr, len)?);
                    }
                }
                (start, len)
            }
            _ => {
                return Err(BoardFault::Unsupported(format!(
                    "SPI1: команда flash {command:#010x} вне модели чтения"
                )));
            }
        };
        self.spi1.buffer[start..start + len].copy_from_slice(&reply[..len]);
        // spi_mem_reg.h: биты CMD сбрасываются при завершении операции.
        Ok(())
    }

    fn readback(&self) -> Result<&EspFlashReadback, BoardFault> {
        self.readback.as_ref().ok_or_else(|| {
            BoardFault::Unsupported("не задан профиль ответов JEDEC/status flash".into())
        })
    }
}

#[derive(Clone)]
struct Cache {
    mmu: [u32; ENTRIES],
    enabled: bool,
    shut: u32,
    sync_addr: u32,
    sync_size: u32,
    clock_control: u32,
    mem_power: [u32; 2],
    mmu_owner: u32,
    illegal: Cell<u32>,
    illegal_enable: u32,
    core_access: Cell<u32>,
    core_enable: u32,
    frozen: bool,
    sram0_cpu: bool,
    sram_locked: bool,
    sram0: Vec<Option<u32>>,
    // Для неизменяемой flash содержимое строки определяется страницей.
    // Если строка могла уцелеть при смене MMU, результат требует инвалидации.
    lines: RefCell<BTreeMap<u32, u32>>,
}

impl Default for Cache {
    fn default() -> Self {
        Self {
            // Начальное недействительное отображение соответствует mmu_ll_unmap_all
            // из ESP-IDF; этот же reset наблюдается на Espressif QEMU.
            mmu: [INVALID; ENTRIES],
            enabled: false,
            shut: SHUT_IBUS | SHUT_DBUS,
            sync_addr: 0,
            sync_size: 0,
            clock_control: ICACHE_CLOCK | DCACHE_CLOCK,
            mem_power: [CACHE_MEM_FORCE_PU | CACHE_MEM_FORCE_ON; 2],
            mmu_owner: 0, // EXTMEM_CACHE_MMU_OWNER_REG, значение сброса.
            illegal: Cell::new(0),
            illegal_enable: 0,
            core_access: Cell::new(0),
            core_enable: 0,
            frozen: false,
            sram0_cpu: true, // TRM v1.4 регистр 14.6: сброс CPU_CACHE=1.
            sram_locked: false,
            sram0: vec![None; ((SRAM0.end - SRAM0.start) / u32::from(WORD)) as usize],
            lines: RefCell::default(),
        }
    }
}

impl Cache {
    fn report_access(&self, bus: u32, ibus: u32, dbus: u32) {
        let flag = if bus == SHUT_IBUS { ibus } else { dbus };
        self.core_access.set(self.core_access.get() | flag);
    }

    fn register(
        &mut self,
        pc: u32,
        addr: u32,
        val: Option<u32>,
        unknown: BoardFault,
    ) -> Result<u32, BoardFault> {
        match addr {
            TAG_POWER | MMU_POWER => {
                if val.is_some_and(|word| word & !CACHE_MEM_FORCE_ON != CACHE_MEM_FORCE_PU) {
                    return Err(unknown);
                }
                // extmem_reg.h: FORCE_ON управляет гейтированием в простое;
                // такты пробуждения памяти отдельно не считаются.
                update(
                    &mut self.mem_power[usize::from(addr == MMU_POWER)],
                    val,
                    CACHE_MEM_FORCE_PU | CACHE_MEM_FORCE_ON,
                )
            }
            SYSTEM_CACHE_CONTROL => {
                if let Some(bits) = val {
                    // TRM §3.3.3: IBUS и DBUS используют общий ICACHE.
                    // В модели допускается только состояние сброса полей DCACHE.
                    if bits & !(ICACHE_CLOCK | ICACHE_RESET) != DCACHE_CLOCK {
                        return Err(unknown);
                    }
                    if bits & ICACHE_RESET != 0 {
                        self.lines.get_mut().clear();
                        self.illegal.set(0);
                        self.illegal_enable = 0;
                        self.core_access.set(0);
                        self.core_enable = 0;
                        self.frozen = false;
                    }
                    self.clock_control = bits;
                    return Ok(0);
                }
                Ok(self.clock_control)
            }
            MMU_OWNER => match val {
                // ROM Cache_MMU_Init включает обе шины перед заполнением
                // таблицы. Другие сочетания владельцев требуют отдельной модели.
                Some(MMU_BOTH_BUSES) => {
                    self.mmu_owner = MMU_BOTH_BUSES;
                    Ok(0)
                }
                None => Ok(self.mmu_owner),
                _ => Err(unknown),
            },
            CACHE_ILG_ENABLE => update(&mut self.illegal_enable, val, CACHE_ILG_W1C),
            CACHE_CORE_ENABLE => update(&mut self.core_enable, val, CACHE_CORE_MASK),
            CACHE_ILG_STATUS if val.is_none() => Ok(self.illegal.get()),
            CACHE_CORE_STATUS if val.is_none() => Ok(self.core_access.get()),
            CACHE_ILG_CLEAR | CACHE_CORE_CLEAR => match val {
                // WOD без сохраняемого значения. SDK использует read-modify-write.
                None => Ok(0),
                // Cache_MMU_Init в ROM rev. 3 очищает все WOD-поля записью
                // всех единиц. Биты без WOD-полей не запускают операций.
                Some(bits)
                    if addr == CACHE_ILG_CLEAR
                        && (bits == u32::MAX || bits & !CACHE_ILG_W1C == 0) =>
                {
                    *self.illegal.get_mut() &= !bits;
                    Ok(0)
                }
                Some(bits) if addr == CACHE_CORE_CLEAR && bits & !CACHE_CORE_MASK == 0 => {
                    *self.core_access.get_mut() &= !bits;
                    Ok(0)
                }
                _ => Err(unknown),
            },
            // EXTMEM_CACHE_FLASH_WRAP_AROUND: снимок читается линейно.
            CACHE_WRAP if matches!(val, None | Some(0)) => Ok(0),
            // AUTOLOAD_ENA=0 при сбросе. Команды фоновой подкачки не реализованы.
            AUTOLOAD_CTRL if val.is_none() => Ok(AUTOLOAD_DONE),
            // PRELOAD_ENA=0 при сбросе; запуск подкачки требует отдельной модели.
            PRELOAD_CTRL if val.is_none() => Ok(PRELOAD_DONE),
            CACHE_FREEZE => {
                if let Some(bits) = val {
                    // extmem_reg.h, EXTMEM_ICACHE_FREEZE_REG: режим 0
                    // без пополнения кэша. Пока он заморожен, проверенный
                    // доступ возможен только из ROM/IRAM: read_cached отклоняет
                    // оба flash-окна, так как состав строк кэша неизвестен.
                    if bits & !(FREEZE_ENABLE | FREEZE_DONE) != 0 {
                        return Err(unknown);
                    }
                    self.frozen = bits & FREEZE_ENABLE != 0;
                    return Ok(0);
                }
                Ok(if self.frozen {
                    FREEZE_ENABLE | FREEZE_DONE
                } else {
                    0
                })
            }
            MMU..MMU_END => {
                let index = ((addr - MMU) / u32::from(WORD)) as usize;
                update(&mut self.mmu[index], val, PAGE_NUMBER | INVALID)
            }
            SRAM_LOCK => {
                if let Some(value) = checked(val, LOCK)? {
                    self.sram_locked |= value & LOCK != 0;
                    return Ok(0);
                }
                Ok(u32::from(self.sram_locked))
            }
            SRAM_USAGE => {
                if let Some(value) = checked(val, SRAM_USAGE_RESERVED | SRAM0_CPU)? {
                    let cpu = value & SRAM0_CPU != 0;
                    if self.sram_locked && cpu != self.sram0_cpu {
                        return Err(BoardFault::Misconfigured(
                            "назначение SRAM0 заблокировано PMS".into(),
                        ));
                    }
                    if cpu != self.sram0_cpu {
                        self.sram0_cpu = cpu;
                        self.sram0.fill(None);
                        // При возврате банка кэшу состояние строк, чьи теги
                        // могли сохраниться, неизвестно до инвалидации.
                        for entry in self.lines.get_mut().values_mut() {
                            *entry = INVALID;
                        }
                    }
                    return Ok(0);
                }
                Ok(SRAM_USAGE_RESERVED | u32::from(self.sram0_cpu))
            }
            CACHE_CTRL => {
                if let Some(value) = checked(val, CACHE_ENABLE)? {
                    self.enabled = value != 0;
                    return Ok(0);
                }
                Ok(u32::from(self.enabled))
            }
            CACHE_BUS => update(&mut self.shut, val, SHUT_IBUS | SHUT_DBUS),
            SYNC_ADDR => update(&mut self.sync_addr, val, u32::MAX),
            SYNC_SIZE => update(&mut self.sync_size, val, SYNC_SIZE_MASK),
            SYNC_CTRL => {
                if let Some(value) = checked(val, INVALIDATE | SYNC_DONE)? {
                    if value & INVALIDATE != 0
                        && let Err(fault) = self.invalidate()
                    {
                        *self.illegal.get_mut() |= SYNC_FAULT;
                        return Err(fault);
                    }
                    return Ok(0);
                }
                // EXTMEM_ICACHE_INVALIDATE_ENA запускает операцию и при
                // сбросе. Здесь инвалидация, включая начальную, атомарна.
                Ok(SYNC_DONE)
            }
            CACHE_STATE => {
                if val.is_some() {
                    return Err(BoardFault::WriteToReadOnly { pc, addr });
                }
                Ok(CACHE_IDLE)
            }
            _ => Err(unknown),
        }
    }

    fn invalidate(&mut self) -> Result<(), BoardFault> {
        // TRM §3.3.3.3: полная инвалидация или по диапазону 32-байтных блоков.
        // Нулевые адрес и длина для полной инвалидации сверены по записи ROM
        // в EXTMEM_ICACHE_SYNC_{ADDR,SIZE,CTRL} на Espressif QEMU.
        if self.sync_addr == 0 && self.sync_size == 0 {
            self.lines.get_mut().clear();
            return Ok(());
        }
        let (linear, _) = cache_address(self.sync_addr)
            .ok_or_else(|| BoardFault::Misconfigured("адрес инвалидации вне DROM/IROM".into()))?;
        let first = linear / LINE;
        let end = first + self.sync_size;
        if self.sync_size == 0 || end > WINDOW_LEN / LINE {
            return Err(BoardFault::Misconfigured(
                "длина инвалидации вне окна кэша".into(),
            ));
        }
        self.lines
            .get_mut()
            .retain(|line, _| !(first..end).contains(line));
        Ok(())
    }
}

#[derive(Clone)]
struct SpiMem {
    addr: u32,
    ctrl: u32,
    ctrl1: u32,
    ctrl2: u32,
    clock: u32,
    clock_gate: u32,
    core_clk_sel: u32,
    user: u32,
    user1: u32,
    user2: u32,
    mosi_dlen: u32,
    miso_dlen: u32,
    status: u32,
    misc: u32,
    cache_fctrl: u32,
    lock_delay: u32,
    wait_idle_ctrl: u32,
    sus_ctrl: u32,
    buffer: [u8; SPI_BYTES],
}

impl Default for SpiMem {
    fn default() -> Self {
        // Значения сброса соответствующих полей в spi_mem_reg.h.
        Self {
            addr: 0,
            ctrl: CTRL_RESET,
            ctrl1: CTRL1_RESET,
            ctrl2: CS_TIME_RESET,
            clock: CLOCK_RESET,
            clock_gate: CLOCK_FORCE_ON,
            core_clk_sel: 0,
            user: USR_COMMAND,
            user1: (23 << ADDR_SHIFT) | 7,
            user2: 7 << COMMAND_SHIFT,
            mosi_dlen: 0,
            miso_dlen: 0,
            status: 0,
            misc: CS1_DIS,
            cache_fctrl: 0,
            lock_delay: LOCK_DELAY_RESET,
            wait_idle_ctrl: WAIT_IDLE_RESET,
            sus_ctrl: SUS_CTRL_RESET,
            buffer: [0; SPI_BYTES],
        }
    }
}

impl SpiMem {
    fn register(
        &mut self,
        offset: u32,
        val: Option<u32>,
        spi0: bool,
        unknown: BoardFault,
    ) -> Result<u32, BoardFault> {
        match offset {
            CMD if val.is_none() => Ok(0), // CMD.MST_ST/SLV_ST: idle после операции.
            ADDR => update(&mut self.addr, val, u32::MAX),
            CTRL => update(
                &mut self.ctrl,
                val,
                CTRL_RESET | FREAD_DIO | FREAD_DUAL | FDUMMY_OUT,
            ),
            CTRL1 => update(&mut self.ctrl1, val, CTRL1_RW),
            CTRL2 => {
                if val.is_some_and(|bits| bits & !(CS_TIME_MASK | SYNC_RESET) != 0) {
                    return Err(unknown);
                }
                // Сброс FSM самосбрасывается. Между атомарными транзакциями FSM в idle.
                update(
                    &mut self.ctrl2,
                    val.map(|bits| bits & !SYNC_RESET),
                    CS_TIME_MASK,
                )
            }
            CLOCK => update(&mut self.clock, val, CLOCK_MASK | CLOCK_SYSCLK),
            // rtc_init.c снимает принудительное тактирование в простое.
            CLOCK_GATE => update(&mut self.clock_gate, val, CLOCK_FORCE_ON),
            // SPIMEM0.core_clk_sel в spimem_flash_ll_get_source_freq_mhz.
            // Транзакции снимка атомарны, частота нужна для расчётов прошивки.
            CORE_CLK_SEL if spi0 && val.is_none_or(|selection| selection < 0b11) => {
                update(&mut self.core_clk_sel, val, 0b11)
            }
            USER => update(
                &mut self.user,
                val,
                USR_COMMAND | USR_ADDR | USR_DUMMY | USR_MISO | USR_MISO_HIGHPART | CS_TIMING,
            ),
            USER1 => update(&mut self.user1, val, (ADDR_LEN << ADDR_SHIFT) | DUMMY_LEN),
            USER2 => update(&mut self.user2, val, (0xf << COMMAND_SHIFT) | COMMAND_VALUE),
            MOSI_DLEN => update(&mut self.mosi_dlen, val, BIT_LEN),
            MISO_DLEN => update(&mut self.miso_dlen, val, BIT_LEN),
            RD_STATUS => update(&mut self.status, val, DATA_STATUS),
            MISC => update(&mut self.misc, val, CS0_DIS | CS1_DIS | CK_IDLE_EDGE),
            CACHE_FCTRL if spi0 => update(
                &mut self.cache_fctrl,
                val,
                CACHE_REQ_EN | CACHE_USR_ADDR_4B | CACHE_USR_CMD,
            ),
            FSM if spi0 => update(&mut self.lock_delay, val, LOCK_DELAY_MASK),
            WAIT_IDLE_CTRL if !spi0 => update(&mut self.wait_idle_ctrl, val, WAIT_IDLE_MASK),
            // spi_flash_hal_disable_auto_suspend_mode в ESP-IDF. Снимок
            // flash допускает настройку выключенного контроллера приостановки.
            SUS_CTRL if !spi0 && val.is_none_or(|word| word & SUS_CTRL_ACTIVE == 0) => {
                update(&mut self.sus_ctrl, val, !SUS_CTRL_ACTIVE)
            }
            // SPI_MEM_FLASH_SUS=0 для снимка без операций программирования/стирания.
            // Остальные поля нулевые при сбросе, арбитраж и команды suspend не поддержаны.
            SUS_STATUS if !spi0 && matches!(val, None | Some(0)) => Ok(0),
            W0..=W15 => {
                let start = (offset - W0) as usize;
                let bytes = &mut self.buffer[start..start + usize::from(WORD)];
                if let Some(val) = val {
                    bytes.copy_from_slice(&val.to_le_bytes());
                    return Ok(0);
                }
                Ok(u32::from_le_bytes(
                    bytes.try_into().expect("одно слово SPI"),
                ))
            }
            _ => Err(unknown),
        }
    }

    fn rx_len(&self, limit: usize) -> Result<usize, BoardFault> {
        let bits = self.miso_dlen + 1;
        let len = (bits / u8::BITS) as usize;
        if !bits.is_multiple_of(u8::BITS) || len > limit {
            return Err(BoardFault::Misconfigured(format!(
                "SPI flash: приём {bits} бит вне байтового буфера на {limit} байт"
            )));
        }
        Ok(len)
    }

    fn opcode(&self) -> Result<u32, BoardFault> {
        let opcode = self.user2 & COMMAND_VALUE;
        if self.user & USR_COMMAND == 0
            || self.user2 >> COMMAND_SHIFT != u8::BITS - 1
            || opcode > u32::from(u8::MAX)
        {
            return Err(BoardFault::Unsupported(
                "SPI flash: требуется 8-битная команда".into(),
            ));
        }
        Ok(opcode)
    }

    fn address_bits(&self) -> u32 {
        (self.user1 >> ADDR_SHIFT) + 1
    }

    fn read_mode(&self) -> u32 {
        if self.ctrl & FASTRD != 0 {
            self.ctrl & (FREAD_DIO | FREAD_DUAL)
        } else {
            0
        }
    }

    fn physical_address(&self, bits: u32) -> Result<u32, BoardFault> {
        // SPI_MEM_ADDR и spimem_flash_ll_set_usr_address: адрес в младших битах.
        match bits {
            24 => Ok(self.addr & (PHYSICAL_LIMIT - 1)),
            32 => Ok(self.addr),
            _ => Err(BoardFault::Unsupported(format!(
                "SPI flash: {bits}-битный адрес"
            ))),
        }
    }

    fn read_address_bits(&self) -> Result<u32, BoardFault> {
        let opcode = self.opcode()?;
        // spi_flash/spi_flash_chip_generic.c и SPI_FLASH_*_DUMMY_BITLEN
        // из spi_flash_defs.h. Такты компенсации сверх обязательных допустимы.
        let (bits, mode, min_dummy) = match opcode {
            READ => (24, 0, 0),
            READ_4B => (32, 0, 0),
            FAST_READ => (24, 0, 8),
            // ESP-IDF bootloader_flash.c: bootloader_flash_read_sfdp.
            RDSFDP => (24, 0, 8),
            FAST_READ_4B => (32, 0, 8),
            DUAL_OUT => (24, FREAD_DUAL, 8),
            DUAL_OUT_4B => (32, FREAD_DUAL, 8),
            DUAL_IO => (24, FREAD_DIO, 4),
            DUAL_IO_4B => (32, FREAD_DIO, 4),
            _ => {
                return Err(BoardFault::Unsupported(format!(
                    "SPI flash: неизвестная команда чтения {opcode:#04x}"
                )));
            }
        };
        let dummy = if self.user & USR_DUMMY != 0 {
            (self.user1 & DUMMY_LEN) + 1
        } else {
            0
        };
        if self.user & (USR_ADDR | USR_MISO) != (USR_ADDR | USR_MISO)
            || self.address_bits() != bits
            || self.read_mode() != mode
            || dummy < min_dummy
        {
            return Err(BoardFault::Misconfigured(format!(
                "SPI flash: неверные фазы команды {opcode:#04x}"
            )));
        }
        Ok(bits)
    }
}

fn range(offset: u32, len: usize, limit: u32) -> Option<Range<usize>> {
    let start = offset as usize;
    let end = start.checked_add(len)?;
    (end <= limit as usize).then_some(start..end)
}

fn cache_address(addr: u32) -> Option<(u32, u32)> {
    if DROM.contains(&addr) {
        Some((addr - DROM.start, SHUT_DBUS))
    } else if IROM.contains(&addr) {
        Some((addr - IROM.start, SHUT_IBUS))
    } else {
        None
    }
}

fn checked(val: Option<u32>, mask: u32) -> Result<Option<u32>, BoardFault> {
    if let Some(value) = val
        && value & !mask != 0
    {
        return Err(BoardFault::Unsupported(format!(
            "запись неподдержанных полей flash {:#010x}",
            value & !mask
        )));
    }
    Ok(val)
}

fn update(dst: &mut u32, val: Option<u32>, mask: u32) -> Result<u32, BoardFault> {
    if let Some(value) = checked(val, mask)? {
        *dst = value;
        Ok(0)
    } else {
        Ok(*dst)
    }
}

fn check_alignment(pc: u32, addr: u32, size: u8) -> Result<(), BoardFault> {
    if !matches!(size, 1 | 2 | WORD) || !addr.is_multiple_of(u32::from(size)) {
        return Err(BoardFault::Unaligned { pc, addr, size });
    }
    Ok(())
}

fn unknown_sram(addr: u32) -> BoardFault {
    BoardFault::Unsupported(format!("SRAM0: содержимое по {addr:#010x} не задано"))
}

fn bus_error(pc: u32, addr: u32, write: bool) -> BoardFault {
    BoardFault::BusError { pc, addr, write }
}
