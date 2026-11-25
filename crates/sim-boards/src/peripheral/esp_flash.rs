//! Функциональная модель чтения flash ESP32-C3. Такты SPI, вытеснение
//! строк кэша, шифрование и запись flash требуют отдельных моделей.

use std::{cell::RefCell, collections::BTreeMap, ops::Range, sync::Arc};

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
const SHUT_IBUS: u32 = 1 << 0;
const SHUT_DBUS: u32 = 1 << 1;
const SYNC_CTRL: u32 = EXTMEM + 0x028;
const INVALIDATE: u32 = 1 << 0;
const SYNC_DONE: u32 = 1 << 1;
const SYNC_ADDR: u32 = EXTMEM + 0x02c;
const SYNC_SIZE: u32 = EXTMEM + 0x030;
const SYNC_SIZE_MASK: u32 = 0x007f_ffff;
const CACHE_STATE: u32 = EXTMEM + 0x0b0;
const CACHE_IDLE: u32 = 1;

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
const CLOCK: u32 = 0x014;
const USER: u32 = 0x018;
const USER1: u32 = 0x01c;
const USER2: u32 = 0x020;
const MOSI_DLEN: u32 = 0x024;
const MISO_DLEN: u32 = 0x028;
const RD_STATUS: u32 = 0x02c;
const MISC: u32 = 0x034;
const CACHE_FCTRL: u32 = 0x03c;
const W0: u32 = 0x058;
const W15: u32 = 0x094;
const SPI_BYTES: usize = (W15 - W0 + WORD as u32) as usize;
const FLASH_READ: u32 = 1 << 31;
const FLASH_RDID: u32 = 1 << 28;
const FLASH_RDSR: u32 = 1 << 27;
const USR: u32 = 1 << 18;
const USR_COMMAND: u32 = 1 << 31;
const USR_ADDR: u32 = 1 << 30;
const USR_DUMMY: u32 = 1 << 29;
const USR_MISO: u32 = 1 << 28;
const USR_MISO_HIGHPART: u32 = 1 << 24;
const CS_TIMING: u32 = (1 << 7) | (1 << 6); // SPI_MEM_CS_SETUP, SPI_MEM_CS_HOLD.
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
const CTRL_RESET: u32 = (1 << 21) | (1 << 19) | (1 << 18) | (1 << 15) | FASTRD;
const CLOCK_MASK: u32 = 0x00ff_ffff; // SPI_MEM_CLKCNT_N/H/L.
const CLOCK_RESET: u32 = 0x0003_0103; // N=3, H=1, L=3.
const CLOCK_SYSCLK: u32 = 1 << 31; // Только SPI0.

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
const RDSR: u32 = 0x05;
const RDSR2: u32 = 0x35;
const STATUS_BUSY_OR_WREN: u8 = (1 << 0) | (1 << 1); // SR_WIP, SR_WREN.

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
        let write = value.is_some();
        if let Some((linear, bus)) = cache_address(addr) {
            check_alignment(pc, addr, size)?;
            if write {
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
        } else if (EXTMEM..MMU_END).contains(&addr) || matches!(addr, SRAM_LOCK | SRAM_USAGE) {
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

        if (SPI1..SPI0).contains(&addr) {
            let offset = addr - SPI1;
            if let (CMD, Some(command)) = (offset, value) {
                self.spi_command(command)?;
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
        if !self.cache.enabled
            || self.cache.shut & bus != 0
            || self.cache.sram0_cpu
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

    fn spi_command(&mut self, command: u32) -> Result<(), BoardFault> {
        if command == 0 {
            return Ok(());
        }
        if self.spi1.misc & CS0_DIS != 0 || self.spi1.misc & CS1_DIS == 0 {
            return Err(BoardFault::Unsupported(
                "SPI1: flash должна быть выбрана на CS0".into(),
            ));
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
                let start = if spi.user & USR_MISO_HIGHPART != 0 {
                    SPI_BYTES / 2
                } else {
                    0
                };
                let len = spi.rx_len(SPI_BYTES - start)?;
                let opcode = spi.opcode()?;
                let phases = spi.user & !(USR_MISO_HIGHPART | CS_TIMING);
                if matches!(opcode, RDID | RDSR | RDSR2) {
                    if phases != USR_COMMAND | USR_MISO {
                        return Err(BoardFault::Misconfigured(
                            "SPI1: неверные фазы RDID/RDSR".into(),
                        ));
                    }
                    let profile = self.readback()?;
                    let status2;
                    let reply_bytes: &[u8] = match opcode {
                        RDID => &profile.jedec_id,
                        RDSR => std::slice::from_ref(&profile.status1),
                        _ => {
                            status2 = profile.status2.ok_or_else(|| {
                                BoardFault::Unsupported("не задан ответ flash на RDSR2".into())
                            })?;
                            std::slice::from_ref(&status2)
                        }
                    };
                    reply[..len].copy_from_slice(reply_bytes.get(..len).ok_or_else(|| {
                        BoardFault::Unsupported("SPI1: длина ответа RDID/RDSR вне профиля".into())
                    })?);
                } else {
                    let bits = spi.read_address_bits()?;
                    let addr = spi.physical_address(bits)?;
                    reply[..len].copy_from_slice(self.read_physical(addr, len)?);
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
            sram0_cpu: true, // TRM v1.4 регистр 14.6: сброс CPU_CACHE=1.
            sram_locked: false,
            sram0: vec![None; ((SRAM0.end - SRAM0.start) / u32::from(WORD)) as usize],
            lines: RefCell::default(),
        }
    }
}

impl Cache {
    fn register(
        &mut self,
        pc: u32,
        addr: u32,
        val: Option<u32>,
        unknown: BoardFault,
    ) -> Result<u32, BoardFault> {
        match addr {
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
                    if value & INVALIDATE != 0 {
                        self.invalidate()?;
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
    clock: u32,
    user: u32,
    user1: u32,
    user2: u32,
    mosi_dlen: u32,
    miso_dlen: u32,
    status: u32,
    misc: u32,
    cache_fctrl: u32,
    buffer: [u8; SPI_BYTES],
}

impl Default for SpiMem {
    fn default() -> Self {
        // Значения сброса соответствующих полей в spi_mem_reg.h.
        Self {
            addr: 0,
            ctrl: CTRL_RESET,
            clock: CLOCK_RESET,
            user: USR_COMMAND,
            user1: (23 << ADDR_SHIFT) | 7,
            user2: 7 << COMMAND_SHIFT,
            mosi_dlen: 0,
            miso_dlen: 0,
            status: 0,
            misc: CS1_DIS,
            cache_fctrl: 0,
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
            CLOCK => update(
                &mut self.clock,
                val,
                CLOCK_MASK | if spi0 { CLOCK_SYSCLK } else { 0 },
            ),
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
