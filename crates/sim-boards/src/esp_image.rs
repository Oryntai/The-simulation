//! Двоичные образы ESP32-C3, которые записываются во flash.
//!
//! Источники формата: ESP-IDF v6.1.1, `components/bootloader_support` и
//! `components/partition_table`. Функции проверяют представленные байты и их
//! размещение. Профиль eFuse, Secure Boot и выбор загрузочного слота здесь не моделируются.

use crate::fault::BoardFault;
use std::ops::Range;

// esp_rom/esp32c3/Kconfig.soc_caps.in, ESP_ROM_BOOTLOADER_OFFSET_FLASH.
pub const BOOTLOADER_OFFSET: u32 = 0;
// partition_table/Kconfig.projbuild, PARTITION_TABLE_OFFSET.
pub const DEFAULT_PARTITION_TABLE_OFFSET: u32 = 0x8000;

// bootloader_support/include/esp_app_format.h.
const HEADER_LEN: usize = 24;
const IMAGE_MAGIC: u8 = 0xe9;
const MAX_SEGMENTS: u8 = 16;
const CHIP_ID: u16 = 0x0005;
const SEGMENT_HEADER_LEN: usize = 8;

// bootloader_support/src/esp_image_format.c, ESP_ROM_CHECKSUM_INITIAL,
// ESP_IMAGE_MAX_FLASH_ADDR_SIZE для C3 и process_checksum.
// Размер SHA-256: bootloader_support/include/esp_image_format.h, ESP_IMAGE_HASH_LEN.
const CHECKSUM_SEED: u8 = 0xef;
const CHECKSUM_ALIGNMENT: usize = 16;
const MAX_IDF_IMAGE_LEN: u32 = 0x0100_0000;
const SHA256_LEN: usize = 32;

// soc/esp32c3/Kconfig.mmu; TRM v1.4, глава System and Memory.
const MMU_PAGE_SIZE: u32 = 0x10000;
// soc/esp32c3/include/soc/ext_mem_defs.h, SOC_MMU_MAX_PADDR_PAGE_NUM.
const MMU_PHYSICAL_LIMIT: u32 = 256 * MMU_PAGE_SIZE;
// soc/esp32c3/include/soc/soc.h, SOC_{DROM,IROM,DRAM,IRAM,RTC_IRAM}_{LOW,HIGH}.
const DROM: Range<u32> = 0x3c00_0000..0x3c80_0000;
const IROM: Range<u32> = 0x4200_0000..0x4280_0000;
const DRAM: Range<u32> = 0x3fc8_0000..0x3fce_0000;
const IRAM: Range<u32> = 0x4037_c000..0x403e_0000;
const RTC: Range<u32> = 0x5000_0000..0x5000_2000;

// bootloader_support/include/esp_flash_partitions.h, esp_partition_info_t,
// ESP_PARTITION_TABLE_{SIZE,MAX_LEN}, ESP_PARTITION_MAGIC{,_MD5}, PART_{TYPE,FLAG}_*.
// partition_table/gen_esp32part.py, ALIGNMENT, MD5_PARTITION_BEGIN.
const SECTOR_SIZE: u32 = 0x1000;
const TABLE_LEN: usize = 0xc00;
const RECORD_LEN: usize = 32;
const PARTITION_MAGIC: [u8; 2] = [0xaa, 0x50];
const MD5_MAGIC: [u8; 2] = [0xeb, 0xeb];
const MD5_OFFSET: usize = 16;
const APP_ALIGNMENT: u32 = 0x10000;
const APP: u8 = 0x00;
const DATA: u8 = 0x01;
const BOOTLOADER: u8 = 0x02;
const PARTITION_TABLE: u8 = 0x03;
const PRIMARY: u8 = 0x00;
const ENCRYPTED: u32 = 1 << 0;
const READ_ONLY: u32 = 1 << 1;
// partition_table/gen_esp32part.py, PartitionTable.{to_binary,from_binary}.
const ERASED: u8 = 0xff;

/// Поля `esp_image_header_t` из `bootloader_support/include/esp_app_format.h`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EspImageHeader {
    pub segment_count: u8,
    /// Код `esp_image_spi_mode_t`.
    pub spi_mode: u8,
    /// Код `esp_image_spi_freq_t` (делитель частоты).
    pub spi_speed: u8,
    /// Ёмкость, объявленная в заголовке образа, в байтах.
    pub flash_size: u32,
    pub entry_addr: u32,
    pub wp_pin: u8,
    pub spi_pin_drv: [u8; 3],
    pub chip_id: u16,
    pub min_chip_rev: u8,
    pub min_chip_rev_full: u16,
    pub max_chip_rev_full: u16,
    pub hash_appended: bool,
}

/// Способ размещения сегмента по `should_load` и `should_map` в `esp_image_format.c`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SegmentKind {
    Padding,
    Reserved,
    Dram,
    Iram,
    Rtc,
    Drom,
    Irom,
}

/// Заголовок сегмента и проверенные байты, взятые из исходного среза.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EspSegment<'a> {
    /// `esp_image_segment_header_t::load_addr`.
    pub load_addr: u32,
    /// Абсолютное смещение тела сегмента во flash.
    pub flash_offset: u32,
    pub kind: SegmentKind,
    /// Длина соответствует `esp_image_segment_header_t::data_len`.
    pub data: &'a [u8],
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EspImage<'a> {
    pub header: EspImageHeader,
    pub flash_offset: u32,
    /// Размер заголовка, сегментов, выравнивания, XOR и SHA-256 при его наличии.
    pub byte_len: usize,
    pub checksum: u8,
    pub sha256: Option<[u8; SHA256_LEN]>,
    pub segments: Vec<EspSegment<'a>>,
}

/// Поля `esp_partition_info_t` из `bootloader_support/include/esp_flash_partitions.h`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Partition {
    pub label: String,
    pub kind: u8,
    pub subtype: u8,
    pub offset: u32,
    pub size: u32,
    /// Флаг шифрования по таблице. Активация шифрования зависит от профиля.
    pub encrypted: bool,
    pub read_only: bool,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PartitionTable {
    pub flash_offset: u32,
    /// Порядок записей в таблице на flash.
    pub partitions: Vec<Partition>,
    /// При отключении `CONFIG_PARTITION_TABLE_MD5` контрольная сумма отсутствует.
    pub md5: Option<[u8; RECORD_LEN - MD5_OFFSET]>,
}

/// Объединённый образ esptool в raw-формате, начиная с физического адреса ноль.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FlashImage<'a> {
    bytes: &'a [u8],
    bootloader: EspImage<'a>,
    table: PartitionTable,
    apps: Vec<(u32, EspImage<'a>)>,
}

impl<'a> FlashImage<'a> {
    /// Байты файла. Ёмкость flash указана в заголовке загрузчика.
    pub fn as_bytes(&self) -> &'a [u8] {
        self.bytes
    }

    pub fn bootloader(&self) -> &EspImage<'a> {
        &self.bootloader
    }

    pub fn partition_table(&self) -> &PartitionTable {
        &self.table
    }

    /// Получает проверенный образ в явно указанном разделе.
    pub fn app_image(&self, label: &str) -> Result<&EspImage<'a>, BoardFault> {
        let partition = self
            .table
            .partitions
            .iter()
            .find(|p| p.label == label)
            .ok_or_else(|| invalid(format!("нет раздела {label:?}")))?;
        if partition.kind != APP {
            return Err(invalid(format!("раздел {label:?} не содержит приложение")));
        }
        self.apps
            .iter()
            .find(|(offset, _)| *offset == partition.offset)
            .map(|(_, image)| image)
            .ok_or_else(|| {
                invalid(format!(
                    "раздел приложения {label:?} стёрт или отсутствует в файле"
                ))
            })
    }
}

/// Проверяет один файл приложения или загрузчика по известному адресу записи во flash.
pub fn parse_image(bytes: &[u8], flash_offset: u32) -> Result<EspImage<'_>, BoardFault> {
    let image = parse_image_prefix(bytes, flash_offset)?;
    if bytes.len() != image.byte_len {
        return Err(unsupported(
            "хвост после образа; подписи и дополненные файлы .bin не поддерживаются",
        ));
    }
    Ok(image)
}

/// Проверяет загрузчик, основную таблицу и все присутствующие образы приложений.
/// Незаписанные слоты допустимы, если в файле есть хотя бы одно приложение.
pub fn parse_flash(bytes: &[u8], table_offset: u32) -> Result<FlashImage<'_>, BoardFault> {
    if table_offset > MAX_IDF_IMAGE_LEN {
        return Err(invalid("раздел загрузчика длиннее предела ESP-IDF"));
    }
    let table_start = table_offset as usize;
    let boot_bytes = bytes
        .get(BOOTLOADER_OFFSET as usize..table_start)
        .ok_or_else(|| invalid("файл обрезан до таблицы разделов"))?;
    let bootloader = parse_flash_slot(boot_bytes, BOOTLOADER_OFFSET)?;
    let capacity = bootloader.header.flash_size;
    if bytes.len() > capacity as usize {
        return Err(invalid(
            "raw-файл длиннее ёмкости flash из заголовка загрузчика",
        ));
    }

    let sector_end = table_start
        .checked_add(SECTOR_SIZE as usize)
        .ok_or_else(|| invalid("переполнение адреса таблицы разделов"))?;
    let table_bytes = bytes
        .get(table_start..sector_end)
        .ok_or_else(|| invalid("сектор таблицы разделов обрезан"))?;
    let table = parse_partition_table(table_bytes, table_offset, capacity)?;

    let mut apps = Vec::new();
    for partition in table.partitions.iter().filter(|p| p.kind == APP) {
        let start = partition.offset as usize;
        let end = (partition.offset + partition.size) as usize;
        // esptool merge-bin без опции --pad-to-size заканчивает файл на последнем входном образе.
        let slot = bytes.get(start..end.min(bytes.len())).unwrap_or(&[]);
        if slot.iter().all(|&byte| byte == ERASED) {
            continue;
        }
        let image = parse_flash_slot(slot, partition.offset).map_err(|fault| {
            let context = format!("раздел {:?} ({:#x})", partition.label, partition.offset);
            match fault {
                BoardFault::BadImage(message) => invalid(format!("{context}: {message}")),
                BoardFault::Unsupported(message) => unsupported(format!("{context}: {message}")),
                other => other,
            }
        })?;
        apps.push((partition.offset, image));
    }
    if apps.is_empty() {
        return Err(invalid("во flash-образе нет записанного приложения"));
    }
    Ok(FlashImage {
        bytes,
        bootloader,
        table,
        apps,
    })
}

fn parse_flash_slot(bytes: &[u8], flash_offset: u32) -> Result<EspImage<'_>, BoardFault> {
    let image = parse_image_prefix(bytes, flash_offset)?;
    if bytes[image.byte_len..].iter().any(|&byte| byte != ERASED) {
        return Err(unsupported(format!(
            "дополнительные байты после образа по адресу {flash_offset:#x}"
        )));
    }
    Ok(image)
}

fn parse_image_prefix(bytes: &[u8], flash_offset: u32) -> Result<EspImage<'_>, BoardFault> {
    // bootloader_support/bootloader_flash/src/bootloader_flash.c, bootloader_flash_read:
    // адрес чтения заголовка должен быть выровнен на слово.
    if !flash_offset.is_multiple_of(4) {
        return Err(invalid(format!(
            "адрес образа {flash_offset:#x} не выровнен"
        )));
    }
    let header = parse_header(bytes)?;
    let mut segments = Vec::with_capacity(header.segment_count as usize);
    let mut cursor = HEADER_LEN;
    let mut checksum = CHECKSUM_SEED;

    for index in 0..header.segment_count {
        let h = take(bytes, &mut cursor, SEGMENT_HEADER_LEN)
            .ok_or_else(|| invalid(format!("сегмент {index}: обрезан заголовок")))?;
        // esp_image_segment_header_t, два последовательных поля uint32_t.
        let load_addr = le32(&h[..4]);
        let len = le32(&h[4..]);
        // esp_image_format.c, verify_segment_header: кратность 4 и предел длины.
        if !len.is_multiple_of(4) || len >= MAX_IDF_IMAGE_LEN {
            return Err(invalid(format!(
                "сегмент {index}: недопустимая длина {len:#x}"
            )));
        }
        let data_offset = flash_addr_at(flash_offset, cursor)?;
        let kind = segment_kind(load_addr, len)?;
        if cursor + len as usize > MAX_IDF_IMAGE_LEN as usize {
            return Err(invalid(format!(
                "сегмент {index}: длина образа превышает предел ESP-IDF"
            )));
        }
        let data = take(bytes, &mut cursor, len as usize)
            .ok_or_else(|| invalid(format!("сегмент {index}: обрезаны данные")))?;
        if matches!(kind, SegmentKind::Drom | SegmentKind::Irom)
            && (data_offset % MMU_PAGE_SIZE != load_addr % MMU_PAGE_SIZE
                || u64::from(data_offset) + u64::from(len) > u64::from(MMU_PHYSICAL_LIMIT))
        {
            return Err(invalid(format!(
                "сегмент {index}: MMU не отображает flash {data_offset:#x} на адрес {load_addr:#x}"
            )));
        }
        checksum = data.iter().fold(checksum, |acc, &byte| acc ^ byte);
        segments.push(EspSegment {
            load_addr,
            flash_offset: data_offset,
            kind,
            data,
        });
    }

    // TRM v1.4, глава ESP-RISC-V CPU: расширение C, адрес инструкции кратен двум.
    if !header.entry_addr.is_multiple_of(2)
        || !segments.iter().any(|seg| {
            matches!(
                seg.kind,
                SegmentKind::Iram | SegmentKind::Irom | SegmentKind::Rtc
            ) && header.entry_addr >= seg.load_addr
                && header.entry_addr - seg.load_addr < seg.data.len() as u32
        })
    {
        return Err(invalid(format!(
            "точка входа {:#x} вне кода образа или не выровнена",
            header.entry_addr
        )));
    }

    // esp_image_format.c, process_checksum: XOR записан последним байтом блока.
    let footer_len = CHECKSUM_ALIGNMENT - cursor % CHECKSUM_ALIGNMENT;
    let hash_len = if header.hash_appended { SHA256_LEN } else { 0 };
    if cursor + footer_len + hash_len > MAX_IDF_IMAGE_LEN as usize {
        return Err(invalid(
            "длина образа с контрольными суммами превышает предел ESP-IDF",
        ));
    }
    let footer = take(bytes, &mut cursor, footer_len)
        .ok_or_else(|| invalid("обрезано выравнивание или XOR образа"))?;
    if footer[footer_len - 1] != checksum {
        return Err(invalid(format!(
            "неверный XOR образа, вычислено {checksum:#04x}"
        )));
    }

    let sha256 = if header.hash_appended {
        let hashed_len = cursor;
        let expected = take(bytes, &mut cursor, SHA256_LEN)
            .ok_or_else(|| invalid("обрезан SHA-256 образа"))?;
        let actual = sha256(&bytes[..hashed_len]);
        if expected != actual {
            return Err(invalid("неверный SHA-256 образа"));
        }
        Some(actual)
    } else {
        None
    };
    flash_addr_at(flash_offset, cursor)?;
    Ok(EspImage {
        header,
        flash_offset,
        byte_len: cursor,
        checksum,
        sha256,
        segments,
    })
}

fn parse_header(bytes: &[u8]) -> Result<EspImageHeader, BoardFault> {
    let h = bytes
        .get(..HEADER_LEN)
        .ok_or_else(|| invalid("обрезан заголовок ESP"))?;
    // Порядок байтов и коды полей: esp_app_format.h, esp_image_header_t и esp_image_spi_*.
    if h[0] != IMAGE_MAGIC {
        return Err(invalid(format!("нет сигнатуры ESP {IMAGE_MAGIC:#04x}")));
    }
    let segment_count = h[1];
    if !(1..=MAX_SEGMENTS).contains(&segment_count) {
        return Err(invalid(format!(
            "недопустимое число сегментов {segment_count}"
        )));
    }
    let spi_mode = h[2];
    let spi_speed = h[3] & 0x0f;
    let size_code = h[3] >> 4;
    if spi_mode > 5 || !matches!(spi_speed, 0..=2 | 0x0f) || size_code > 7 {
        return Err(unsupported(format!(
            "неизвестные параметры SPI в заголовке ({spi_mode:#x}, {spi_speed:#x}, {size_code:#x})"
        )));
    }
    // bootloader_flash_config_esp32c3.c, update_flash_config:
    // код ёмкости 0..7 обозначает 1..128 МиБ.
    let flash_size = 0x10_0000 << size_code;
    let chip_id = le16(&h[12..14]);
    if chip_id != CHIP_ID {
        return Err(invalid(format!(
            "образ для другого чипа: ID {chip_id:#06x}, нужен {CHIP_ID:#06x}"
        )));
    }
    if h[19..23] != [0; 4] || h[23] > 1 {
        return Err(unsupported("неизвестное расширение заголовка ESP"));
    }
    Ok(EspImageHeader {
        segment_count,
        spi_mode,
        spi_speed,
        flash_size,
        entry_addr: le32(&h[4..8]),
        wp_pin: h[8],
        spi_pin_drv: [h[9], h[10], h[11]],
        chip_id,
        min_chip_rev: h[14],
        min_chip_rev_full: le16(&h[15..17]),
        max_chip_rev_full: le16(&h[17..19]),
        hash_appended: h[23] == 1,
    })
}

fn segment_kind(addr: u32, len: u32) -> Result<SegmentKind, BoardFault> {
    // esp_image_format.c, should_load: специальные адреса 0 (padding), 4 (MD5, зарезервирован).
    match addr {
        0 => return Ok(SegmentKind::Padding),
        4 => return Ok(SegmentKind::Reserved),
        _ => {}
    }
    let end = addr.checked_add(len).ok_or_else(|| {
        invalid(format!(
            "адресный диапазон сегмента {addr:#x}+{len:#x} переполнен"
        ))
    })?;
    let windows = [
        (DROM, SegmentKind::Drom),
        (IROM, SegmentKind::Irom),
        (DRAM, SegmentKind::Dram),
        (IRAM, SegmentKind::Iram),
        (RTC, SegmentKind::Rtc),
    ];
    windows
        .into_iter()
        .find(|(range, _)| range.contains(&addr) && end <= range.end)
        .map(|(_, kind)| kind)
        .ok_or_else(|| {
            invalid(format!(
                "сегмент {addr:#x}..{end:#x} вне области загрузки C3"
            ))
        })
}

/// Разбирает отдельное содержимое таблицы (0xc00 байт) или весь её сектор.
/// `flash_size` — ёмкость flash в байтах. Она может превышать размер записанного файла.
pub fn parse_partition_table(
    bytes: &[u8],
    table_offset: u32,
    flash_size: u32,
) -> Result<PartitionTable, BoardFault> {
    let table_end = table_offset
        .checked_add(SECTOR_SIZE)
        .filter(|&end| end <= flash_size)
        .ok_or_else(|| invalid("таблица разделов выходит за пределы flash"))?;
    if table_offset <= BOOTLOADER_OFFSET || !table_offset.is_multiple_of(SECTOR_SIZE) {
        return Err(invalid(format!(
            "недопустимое смещение таблицы разделов {table_offset:#x}"
        )));
    }
    if bytes.len() != TABLE_LEN && bytes.len() != SECTOR_SIZE as usize {
        return Err(invalid(format!(
            "неверная длина таблицы разделов {:#x}",
            bytes.len()
        )));
    }
    if bytes[TABLE_LEN..].iter().any(|&byte| byte != ERASED) {
        return Err(unsupported(
            "дополнительные данные в секторе таблицы разделов",
        ));
    }

    let records = &bytes[..TABLE_LEN];
    let mut partitions: Vec<Partition> = Vec::new();
    let mut digest = None;
    for (index, record) in records.chunks_exact(RECORD_LEN).enumerate() {
        let offset = index * RECORD_LEN;
        if record.iter().all(|&byte| byte == ERASED) {
            if records[offset..].iter().any(|&byte| byte != ERASED) {
                return Err(invalid("данные после конца таблицы разделов"));
            }
            if partitions.is_empty() {
                return Err(invalid("пустая таблица разделов"));
            }
            return Ok(PartitionTable {
                flash_offset: table_offset,
                partitions,
                md5: digest,
            });
        }
        if digest.is_some() {
            return Err(invalid("ожидался конец таблицы разделов после MD5"));
        }
        if record.starts_with(&MD5_MAGIC) {
            if record[MD5_MAGIC.len()..MD5_OFFSET]
                .iter()
                .any(|&byte| byte != ERASED)
            {
                return Err(invalid("повреждён маркер MD5 таблицы разделов"));
            }
            let actual = md5(&records[..offset]);
            if record[MD5_OFFSET..] != actual {
                return Err(invalid("неверный MD5 таблицы разделов"));
            }
            digest = Some(actual);
        } else {
            let partition = parse_partition(record, table_offset, table_end, flash_size)?;
            for other in &partitions {
                if partition.label == other.label {
                    return Err(invalid(format!(
                        "повтор имени раздела {:?}",
                        partition.label
                    )));
                }
                if partition.offset < other.offset + other.size
                    && other.offset < partition.offset + partition.size
                {
                    return Err(invalid(format!(
                        "разделы {:?} и {:?} перекрываются",
                        other.label, partition.label
                    )));
                }
                // bootloader_support/src/bootloader_utility.c, bootloader_utility_load_partition_table:
                // слот определяется подтипом приложения, otadata имеет единственную позицию.
                if partition.kind == other.kind
                    && partition.subtype == other.subtype
                    && (partition.kind == APP
                        || (partition.kind == DATA && partition.subtype == 0x00))
                {
                    return Err(invalid(format!(
                        "повтор загрузочного подтипа {:#x}",
                        partition.subtype
                    )));
                }
            }
            partitions.push(partition);
        }
    }
    Err(invalid("нет конца таблицы разделов"))
}

fn parse_partition(
    record: &[u8],
    table_offset: u32,
    table_end: u32,
    flash_size: u32,
) -> Result<Partition, BoardFault> {
    if !record.starts_with(&PARTITION_MAGIC) {
        return Err(invalid("неверная сигнатура записи таблицы разделов"));
    }
    // Смещения полей: esp_flash_partitions.h, esp_partition_info_t.
    let kind = record[2];
    let subtype = record[3];
    let offset = le32(&record[4..8]);
    let size = le32(&record[8..12]);
    let label = &record[12..28];
    let label = label.split(|&byte| byte == 0).next().unwrap_or(label);
    let label = std::str::from_utf8(label).map_err(|_| invalid("имя раздела не UTF-8"))?;
    if label.is_empty() {
        return Err(invalid("пустое имя раздела"));
    }
    let flags = le32(&record[28..32]);
    // Типы и подтипы: esp_flash_partitions.h, PART_TYPE_* и PART_SUBTYPE_*;
    // docs/en/api-guides/partition-tables.rst, Type Field.
    let known = match kind {
        APP => matches!(subtype, 0x00 | 0x10..=0x1f | 0x20),
        BOOTLOADER => matches!(subtype, 0x00..=0x02),
        PARTITION_TABLE => matches!(subtype, 0x00..=0x01),
        DATA | 0x40..=0xfe => true, // в данных возможны дополнительные подтипы приложения
        _ => false,
    };
    if !known {
        return Err(unsupported(format!(
            "раздел {label:?}: неизвестный тип или подтип {kind:#04x}/{subtype:#04x}"
        )));
    }
    if flags & !(ENCRYPTED | READ_ONLY) != 0 {
        return Err(unsupported(format!(
            "раздел {label:?}: неизвестные флаги {flags:#x}"
        )));
    }
    // gen_esp32part.py, PartitionDefinition.verify: readonly только для data,
    // за исключением ota (0x00) и coredump (0x03).
    let read_only = flags & READ_ONLY != 0;
    if read_only && (kind != DATA || matches!(subtype, 0x00 | 0x03)) {
        return Err(invalid(format!(
            "раздел {label:?}: недопустим флаг readonly"
        )));
    }

    // gen_esp32part.py: ALIGNMENT, get_alignment_size_for_type и parse_{address,size}.
    let alignment = if kind == APP {
        APP_ALIGNMENT
    } else {
        SECTOR_SIZE
    };
    if !offset.is_multiple_of(alignment) || (kind == APP && !size.is_multiple_of(SECTOR_SIZE)) {
        return Err(invalid(format!("раздел {label:?} не выровнен по flash")));
    }
    // esp_image_format.c, esp_image_load: размер переданного раздела ограничен.
    if kind == APP && size > MAX_IDF_IMAGE_LEN {
        return Err(invalid(format!("раздел {label:?} длиннее предела ESP-IDF")));
    }
    if size == 0 || offset.checked_add(size).is_none_or(|end| end > flash_size) {
        return Err(invalid(format!(
            "раздел {label:?} имеет пустой диапазон или выходит за пределы flash"
        )));
    }
    let in_layout = match (kind, subtype) {
        (BOOTLOADER, PRIMARY) => offset == BOOTLOADER_OFFSET && offset + size == table_offset,
        (PARTITION_TABLE, PRIMARY) => offset == table_offset && size == SECTOR_SIZE,
        _ => offset >= table_end,
    };
    if !in_layout {
        return Err(invalid(format!(
            "раздел {label:?} нарушает размещение загрузчика и таблицы"
        )));
    }
    Ok(Partition {
        label: label.to_owned(),
        kind,
        subtype,
        offset,
        size,
        encrypted: flags & ENCRYPTED != 0,
        read_only,
    })
}

fn take<'a>(bytes: &'a [u8], cursor: &mut usize, len: usize) -> Option<&'a [u8]> {
    let end = cursor.checked_add(len)?;
    let result = bytes.get(*cursor..end)?;
    *cursor = end;
    Some(result)
}

fn flash_addr_at(base: u32, offset: usize) -> Result<u32, BoardFault> {
    u32::try_from(offset)
        .ok()
        .and_then(|offset| base.checked_add(offset))
        .ok_or_else(|| invalid("переполнение физического адреса flash"))
}

fn le16(bytes: &[u8]) -> u16 {
    u16::from_le_bytes(bytes.try_into().expect("поле из двух байтов"))
}

fn le32(bytes: &[u8]) -> u32 {
    u32::from_le_bytes(bytes.try_into().expect("поле из четырёх байтов"))
}

fn invalid(message: impl Into<String>) -> BoardFault {
    BoardFault::BadImage(message.into())
}

fn unsupported(message: impl Into<String>) -> BoardFault {
    BoardFault::Unsupported(message.into())
}

// FIPS 180-4 §5.1.1 и RFC 1321 §3.1–3.2. Длина в битах кодируется в
// последнем поле, порядок байтов задаётся вызывающим алгоритмом.
// Сверка с ESP-IDF: mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/{md5,sha256}.c.
fn digest_blocks(bytes: &[u8], bit_len: [u8; 8]) -> impl Iterator<Item = [u8; 64]> + '_ {
    let chunks = bytes.chunks_exact(64);
    let remainder = chunks.remainder();
    let mut tail = [0; 64];
    tail[..remainder.len()].copy_from_slice(remainder);
    tail[remainder.len()] = 0x80;
    let mut extra = None;
    if remainder.len() < 56 {
        tail[56..].copy_from_slice(&bit_len);
    } else {
        let mut block = [0; 64];
        block[56..].copy_from_slice(&bit_len);
        extra = Some(block);
    }
    chunks
        .map(|chunk| chunk.try_into().expect("блок хеша"))
        .chain([tail])
        .chain(extra)
}

// FIPS 180-4 §4.1.2, §4.2.2, §5.3.3, §6.2; ESP-IDF, mbedtls/mbedtls/
// tf-psa-crypto/drivers/builtin/src/sha256.c. Используется для целостности, без аутентификации.
fn sha256(bytes: &[u8]) -> [u8; SHA256_LEN] {
    const ROUND: [u32; 64] = [
        0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4,
        0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe,
        0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f,
        0x4a7484aa, 0x5cb0a9dc, 0x76f988da, 0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
        0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc,
        0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b,
        0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116,
        0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
        0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7,
        0xc67178f2,
    ];
    let mut state: [u32; 8] = [
        0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab,
        0x5be0cd19,
    ];
    for block in digest_blocks(bytes, (bytes.len() as u64 * 8).to_be_bytes()) {
        let mut words = [0u32; 64];
        for (word, chunk) in words.iter_mut().zip(block.chunks_exact(4)) {
            *word = u32::from_be_bytes(chunk.try_into().expect("слово SHA-256"));
        }
        for i in 16..words.len() {
            let x = words[i - 15];
            let y = words[i - 2];
            let sigma0 = x.rotate_right(7) ^ x.rotate_right(18) ^ (x >> 3);
            let sigma1 = y.rotate_right(17) ^ y.rotate_right(19) ^ (y >> 10);
            words[i] = words[i - 16]
                .wrapping_add(sigma0)
                .wrapping_add(words[i - 7])
                .wrapping_add(sigma1);
        }

        let [mut a, mut b, mut c, mut d, mut e, mut f, mut g, mut h] = state;
        for (key, word) in ROUND.into_iter().zip(words) {
            let sigma1 = e.rotate_right(6) ^ e.rotate_right(11) ^ e.rotate_right(25);
            let choose = (e & f) ^ (!e & g);
            let t1 = h
                .wrapping_add(sigma1)
                .wrapping_add(choose)
                .wrapping_add(key)
                .wrapping_add(word);
            let sigma0 = a.rotate_right(2) ^ a.rotate_right(13) ^ a.rotate_right(22);
            let majority = (a & b) ^ (a & c) ^ (b & c);
            let t2 = sigma0.wrapping_add(majority);
            [a, b, c, d, e, f, g, h] = [t1.wrapping_add(t2), a, b, c, d.wrapping_add(t1), e, f, g];
        }
        for (word, value) in state.iter_mut().zip([a, b, c, d, e, f, g, h]) {
            *word = word.wrapping_add(value);
        }
    }
    let mut result = [0; SHA256_LEN];
    for (chunk, word) in result.chunks_exact_mut(4).zip(state) {
        chunk.copy_from_slice(&word.to_be_bytes());
    }
    result
}

// RFC 1321 §3.3–3.5; ESP-IDF, mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/src/md5.c.
// MD5 в таблице является контрольной суммой формата.
fn md5(bytes: &[u8]) -> [u8; RECORD_LEN - MD5_OFFSET] {
    const ROUND: [u32; 64] = [
        0xd76aa478, 0xe8c7b756, 0x242070db, 0xc1bdceee, 0xf57c0faf, 0x4787c62a, 0xa8304613,
        0xfd469501, 0x698098d8, 0x8b44f7af, 0xffff5bb1, 0x895cd7be, 0x6b901122, 0xfd987193,
        0xa679438e, 0x49b40821, 0xf61e2562, 0xc040b340, 0x265e5a51, 0xe9b6c7aa, 0xd62f105d,
        0x02441453, 0xd8a1e681, 0xe7d3fbc8, 0x21e1cde6, 0xc33707d6, 0xf4d50d87, 0x455a14ed,
        0xa9e3e905, 0xfcefa3f8, 0x676f02d9, 0x8d2a4c8a, 0xfffa3942, 0x8771f681, 0x6d9d6122,
        0xfde5380c, 0xa4beea44, 0x4bdecfa9, 0xf6bb4b60, 0xbebfbc70, 0x289b7ec6, 0xeaa127fa,
        0xd4ef3085, 0x04881d05, 0xd9d4d039, 0xe6db99e5, 0x1fa27cf8, 0xc4ac5665, 0xf4292244,
        0x432aff97, 0xab9423a7, 0xfc93a039, 0x655b59c3, 0x8f0ccc92, 0xffeff47d, 0x85845dd1,
        0x6fa87e4f, 0xfe2ce6e0, 0xa3014314, 0x4e0811a1, 0xf7537e82, 0xbd3af235, 0x2ad7d2bb,
        0xeb86d391,
    ];
    const ROTATE: [[u32; 4]; 4] = [
        [7, 12, 17, 22],
        [5, 9, 14, 20],
        [4, 11, 16, 23],
        [6, 10, 15, 21],
    ];
    let mut state: [u32; 4] = [0x67452301, 0xefcdab89, 0x98badcfe, 0x10325476];
    for block in digest_blocks(bytes, (bytes.len() as u64 * 8).to_le_bytes()) {
        let mut words = [0u32; 16];
        for (word, chunk) in words.iter_mut().zip(block.chunks_exact(4)) {
            *word = le32(chunk);
        }
        let [mut a, mut b, mut c, mut d] = state;
        for (i, key) in ROUND.into_iter().enumerate() {
            let round = i / 16;
            let (value, word_index) = match round {
                0 => ((b & c) | (!b & d), i),
                1 => ((b & d) | (c & !d), (5 * i + 1) % 16),
                2 => (b ^ c ^ d, (3 * i + 5) % 16),
                _ => (c ^ (b | !d), (7 * i) % 16),
            };
            let mixed = a
                .wrapping_add(value)
                .wrapping_add(words[word_index])
                .wrapping_add(key);
            [a, b, c, d] = [
                d,
                b.wrapping_add(mixed.rotate_left(ROTATE[round][i % 4])),
                b,
                c,
            ];
        }
        for (word, value) in state.iter_mut().zip([a, b, c, d]) {
            *word = word.wrapping_add(value);
        }
    }
    let mut result = [0; RECORD_LEN - MD5_OFFSET];
    for (chunk, word) in result.chunks_exact_mut(4).zip(state) {
        chunk.copy_from_slice(&word.to_le_bytes());
    }
    result
}
