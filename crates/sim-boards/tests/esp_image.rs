//! Проверки парсера ESP32-C3. Внешняя сверка читает результаты штатной сборки SDK.

mod fault {
    pub use sim_boards::BoardFault;
}

// До подключения в Т2.10 модуль входит в закрытый image крейта.
#[path = "../src/esp_image.rs"]
mod esp_image;

use esp_image::{
    BOOTLOADER_OFFSET, DEFAULT_PARTITION_TABLE_OFFSET as TABLE_OFFSET, EspImage, SegmentKind,
    parse_flash, parse_image, parse_partition_table,
};
use fault::BoardFault;
use std::path::Path;
use std::process::Command;

// Смещения формата: esp_app_format.h, esp_image_format.c и gen_esp32part.py.
// Адреса и ёмкость стенда выбраны внутри документированных диапазонов ESP32-C3.
const APP_OFFSET: u32 = 0x10000;
const FLASH_SIZE: u32 = 0x20_0000;
const IRAM_ENTRY: u32 = 0x4038_0000;
const IROM_ENTRY: u32 = 0x4200_002c;
const TABLE_LEN: usize = 0xc00;
const RECORD_LEN: usize = 32;

// Независимо вычисленные Python hashlib для байтов стенда ниже.
const BLOCK_SHA: &str = "6f11ec09480281c0a73f66c10d69c355f183811e6b7144f24162c00351059ffc";
const BLOCK_MD5: &str = "9f501bfbf048b8ffd9a4dfc91306d012";

fn image(entry: u32, segments: &[(u32, &[u8])], sha: Option<&str>) -> Vec<u8> {
    let mut bytes = vec![0xe9, segments.len() as u8, 2, 0x1f]; // DIO, 80 МГц, 2 МиБ
    bytes.extend_from_slice(&entry.to_le_bytes());
    bytes.extend_from_slice(&[
        0xee,
        0,
        0,
        0,
        5,
        0,
        3,
        3,
        0,
        199,
        0,
        0,
        0,
        0,
        0,
        u8::from(sha.is_some()),
    ]);
    let mut xor = 0xef;
    for (addr, data) in segments {
        bytes.extend_from_slice(&addr.to_le_bytes());
        bytes.extend_from_slice(&(data.len() as u32).to_le_bytes());
        bytes.extend_from_slice(data);
        for &byte in *data {
            xor ^= byte;
        }
    }
    bytes.resize(bytes.len() / 16 * 16 + 15, 0);
    bytes.push(xor);
    if let Some(hex) = sha {
        for i in (0..hex.len()).step_by(2) {
            bytes.push(u8::from_str_radix(&hex[i..i + 2], 16).unwrap());
        }
    }
    bytes
}

fn block_image() -> Vec<u8> {
    let payload: Vec<u8> = (0..32).collect();
    image(
        IROM_ENTRY,
        &[(0, &[0; 4]), (IROM_ENTRY, &payload)],
        Some(BLOCK_SHA),
    )
}

fn image_without_hash() -> Vec<u8> {
    // RISC-V NOP (addi x0, x0, 0), машинный код по спецификации RV32I.
    image(IRAM_ENTRY, &[(IRAM_ENTRY, &[0x13, 0, 0, 0])], None)
}

fn record(
    kind: u8,
    subtype: u8,
    offset: u32,
    size: u32,
    label: &str,
    flags: u32,
) -> [u8; RECORD_LEN] {
    let mut raw = [0; RECORD_LEN];
    raw[..4].copy_from_slice(&[0xaa, 0x50, kind, subtype]);
    raw[4..8].copy_from_slice(&offset.to_le_bytes());
    raw[8..12].copy_from_slice(&size.to_le_bytes());
    raw[12..12 + label.len()].copy_from_slice(label.as_bytes());
    raw[28..].copy_from_slice(&flags.to_le_bytes());
    raw
}

fn block_record() -> [u8; RECORD_LEN] {
    record(0, 0, APP_OFFSET, 0x1000, "blok", 0)
}

fn table(records: &[[u8; RECORD_LEN]]) -> Vec<u8> {
    let mut bytes: Vec<_> = records.iter().flatten().copied().collect();
    bytes.resize(TABLE_LEN, 0xff);
    bytes
}

fn table_with_md5() -> Vec<u8> {
    let mut bytes = table(&[block_record()]);
    bytes[RECORD_LEN..RECORD_LEN + 2].copy_from_slice(&[0xeb, 0xeb]);
    for (i, byte) in bytes[RECORD_LEN + 16..2 * RECORD_LEN]
        .iter_mut()
        .enumerate()
    {
        *byte = u8::from_str_radix(&BLOCK_MD5[2 * i..2 * i + 2], 16).unwrap();
    }
    bytes
}

fn merged(table_offset: u32, partition_table: &[u8], application: &[u8]) -> Vec<u8> {
    let mut bytes = image_without_hash();
    bytes.resize(table_offset as usize, 0xff);
    bytes.extend_from_slice(partition_table);
    bytes.resize(APP_OFFSET as usize, 0xff);
    bytes.extend_from_slice(application);
    bytes
}

fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|byte| format!("{byte:02x}")).collect()
}

fn assert_bad<T: std::fmt::Debug>(result: Result<T, BoardFault>, message: &str) {
    assert!(
        matches!(&result, Err(BoardFault::BadImage(actual)) if actual.contains(message)),
        "{result:?}"
    );
}

#[test]
fn блок_поля_смещения_и_целостность() {
    let bytes = block_image();
    let info = parse_image(&bytes, APP_OFFSET).unwrap();
    assert_eq!(info.byte_len, 112);
    assert_eq!(info.flash_offset, APP_OFFSET);
    assert_eq!(info.header.segment_count, 2);
    assert_eq!(info.header.entry_addr, IROM_ENTRY);
    assert_eq!(
        (
            info.header.spi_mode,
            info.header.spi_speed,
            info.header.flash_size
        ),
        (2, 0xf, FLASH_SIZE)
    );
    assert_eq!(
        (info.header.wp_pin, info.header.spi_pin_drv),
        (0xee, [0; 3])
    );
    assert_eq!((info.header.chip_id, info.header.min_chip_rev), (5, 3));
    assert_eq!(
        (info.header.min_chip_rev_full, info.header.max_chip_rev_full),
        (3, 199)
    );
    assert!(info.header.hash_appended);
    assert_eq!(info.checksum, 0xef);
    assert_eq!(hex(&info.sha256.unwrap()), BLOCK_SHA);
    assert_eq!(info.segments[0].kind, SegmentKind::Padding);
    assert_eq!(info.segments[0].flash_offset, APP_OFFSET + 32);
    assert_eq!(info.segments[0].data, [0; 4]);
    assert_eq!(info.segments[1].kind, SegmentKind::Irom);
    assert_eq!(info.segments[1].load_addr, IROM_ENTRY);
    assert_eq!(info.segments[1].flash_offset, APP_OFFSET + 44);
    assert_eq!(info.segments[1].data, (0..32).collect::<Vec<u8>>());
    assert_eq!(info.segments[1].data, &bytes[44..76]);
}

#[test]
fn образ_без_sha_читает_записанный_xor() {
    let bytes = image_without_hash();
    let info = parse_image(&bytes, BOOTLOADER_OFFSET).unwrap();
    assert_eq!(info.byte_len, 48);
    assert_eq!(info.checksum, 0xef ^ 0x13);
    assert_eq!(info.segments[0].kind, SegmentKind::Iram);
    assert!(!info.header.hash_appended);
    assert_eq!(info.sha256, None);
}

#[test]
fn каждая_обрезка_файла_отвергнута() {
    let bytes = block_image();
    for len in 0..bytes.len() {
        assert!(
            matches!(
                parse_image(&bytes[..len], APP_OFFSET),
                Err(BoardFault::BadImage(_))
            ),
            "длина {len}"
        );
    }
}

#[test]
fn портятся_xor_и_sha_независимо() {
    let good = block_image();
    let mut bad = good.clone();
    bad[44] ^= 1;
    assert_bad(parse_image(&bad, APP_OFFSET), "неверный XOR");
    bad[45] ^= 1; // XOR восстановился, SHA-256 должен заметить два изменённых байта.
    assert_bad(parse_image(&bad, APP_OFFSET), "неверный SHA-256");
    let mut bad = good.clone();
    bad[8] ^= 1; // Поле WP находится за пределами области XOR.
    assert_bad(parse_image(&bad, APP_OFFSET), "неверный SHA-256");
    let mut bad = good;
    bad[111] ^= 1;
    assert_bad(parse_image(&bad, APP_OFFSET), "неверный SHA-256");
}

#[test]
fn ошибочные_поля_заголовка_и_сегмента_отвергнуты() {
    let good = block_image();
    for (index, value, message) in [
        (0, 0, "сигнатуры"),
        (1, 0, "число сегментов"),
        (1, 17, "число сегментов"),
        (4, 0x2d, "точка входа"),
        (12, 9, "другого чипа"),
        (28, 5, "недопустимая длина"),
        (40, 0xfc, "обрезаны данные"),
    ] {
        let mut bad = good.clone();
        bad[index] = value;
        assert_bad(parse_image(&bad, APP_OFFSET), message);
    }
    for (index, value) in [(2, 6), (3, 0x13), (3, 0x8f), (19, 1), (23, 2)] {
        let mut bad = good.clone();
        bad[index] = value;
        assert!(matches!(
            parse_image(&bad, APP_OFFSET),
            Err(BoardFault::Unsupported(_))
        ));
    }
    assert_bad(parse_image(&good, APP_OFFSET + 4), "MMU");
    assert_bad(parse_image(&good, APP_OFFSET + 1), "не выровнен");
    assert_bad(parse_image(&good, 0x100_0000 + APP_OFFSET), "MMU");
    assert_bad(
        parse_image(&image_without_hash(), u32::MAX - 3),
        "переполнение физического адреса",
    );
    let mut bad = good.clone();
    // ESP_IMAGE_MAX_FLASH_ADDR_SIZE ограничивает длину сегмента и всего образа.
    bad[28..32].copy_from_slice(&0x00ff_fffc_u32.to_le_bytes());
    assert_bad(parse_image(&bad, APP_OFFSET), "длина образа превышает");
    let mut bad = good.clone();
    bad[36..40].copy_from_slice(&0xffff_fff0_u32.to_le_bytes());
    assert_bad(parse_image(&bad, APP_OFFSET), "переполнен");
    let mut bad = good;
    bad[36..40].copy_from_slice(&0x427f_fff0_u32.to_le_bytes());
    assert_bad(parse_image(&bad, APP_OFFSET), "вне области загрузки");
}

#[test]
fn таблица_md5_и_явное_отсутствие_md5() {
    let checked = table_with_md5();
    let info = parse_partition_table(&checked, TABLE_OFFSET, FLASH_SIZE).unwrap();
    assert_eq!(info.flash_offset, TABLE_OFFSET);
    assert_eq!(hex(&info.md5.unwrap()), BLOCK_MD5);
    assert_eq!(info.partitions.len(), 1);
    let part = &info.partitions[0];
    assert_eq!(
        (
            &*part.label,
            part.kind,
            part.subtype,
            part.offset,
            part.size
        ),
        ("blok", 0, 0, APP_OFFSET, 0x1000)
    );
    assert!(!part.read_only && !part.encrypted);
    assert_eq!(
        parse_partition_table(&table(&[block_record()]), TABLE_OFFSET, FLASH_SIZE)
            .unwrap()
            .md5,
        None
    );

    for (index, message) in [
        (12, "неверный MD5"),
        (34, "маркер MD5"),
        (62, "неверный MD5"),
        (96, "после конца"),
    ] {
        let mut bad = checked.clone();
        bad[index] ^= 1;
        assert_bad(
            parse_partition_table(&bad, TABLE_OFFSET, FLASH_SIZE),
            message,
        );
    }
    let mut bad = checked;
    bad[64..96].copy_from_slice(&block_record());
    assert_bad(
        parse_partition_table(&bad, TABLE_OFFSET, FLASH_SIZE),
        "после MD5",
    );
}

#[test]
fn таблица_проверяет_записи_и_всю_геометрию() {
    let fixtures = [
        (record(1, 2, APP_OFFSET, 0x1000, "data", 0), "перекрываются"),
        (record(1, 2, 0xa000, 0x1000, "blok", 0), "повтор имени"),
        (record(1, 2, 0x8000, 0x1000, "data", 0), "размещение"),
        (record(1, 2, 0x9001, 0x1000, "data", 0), "не выровнен"),
        (
            record(1, 2, 0xffff_f000, 0x2000, "data", 0),
            "пределы flash",
        ),
        (record(1, 2, FLASH_SIZE, 0x1000, "data", 0), "пределы flash"),
        (record(1, 2, 0x9000, 0, "data", 0), "пустой диапазон"),
        (record(0, 0x10, 0x21000, 0x1000, "ota_0", 0), "не выровнен"),
        (record(0, 0x10, 0x20000, 0x1001, "ota_0", 0), "не выровнен"),
        (
            record(0, 0, 0x20000, 0x1000, "повтор", 0),
            "повтор загрузочного подтипа",
        ),
        (record(1, 0, 0x9000, 0x1000, "otadata", 2), "флаг readonly"),
    ];
    for (entry, message) in fixtures {
        assert_bad(
            parse_partition_table(&table(&[block_record(), entry]), TABLE_OFFSET, FLASH_SIZE),
            message,
        );
    }
    let big_app = record(0, 0, APP_OFFSET, 0x100_1000, "blok", 0);
    assert_bad(
        parse_partition_table(&table(&[big_app]), TABLE_OFFSET, 0x200_0000),
        "длиннее предела ESP-IDF",
    );
    let entries = [
        record(2, 0, 0, TABLE_OFFSET, "bootloader", 0),
        record(3, 0, TABLE_OFFSET, 0x1000, "partition_table", 0),
        record(1, 2, 0xa000, 0x1000, "данные", 3),
        record(0x41, 0x42, 0x9000, 0x1000, "custom", 0),
    ];
    let t = parse_partition_table(&table(&entries), TABLE_OFFSET, FLASH_SIZE).unwrap();
    assert_eq!(t.partitions.len(), 4);
    assert_eq!(t.partitions[2].label, "данные");
    assert!(t.partitions[2].read_only && t.partitions[2].encrypted);
    assert_eq!(
        (t.partitions[3].kind, t.partitions[3].subtype),
        (0x41, 0x42)
    );
    let unknown = table(&[record(1, 2, 0x9000, 0x1000, "data", 4)]);
    assert!(matches!(
        parse_partition_table(&unknown, TABLE_OFFSET, FLASH_SIZE),
        Err(BoardFault::Unsupported(_))
    ));
}

#[test]
fn таблица_требует_терминатор_и_полную_длину() {
    let regular = table(&[block_record()]);
    assert_bad(
        parse_partition_table(&regular[..128], TABLE_OFFSET, FLASH_SIZE),
        "длина",
    );
    assert_bad(
        parse_partition_table(&regular, TABLE_OFFSET + 1, FLASH_SIZE),
        "смещение",
    );
    assert_bad(
        parse_partition_table(&regular, TABLE_OFFSET, TABLE_OFFSET),
        "пределы flash",
    );

    // 96 различных корректных записей заполняют всю область 0xc00, места под конец нет.
    let records: Vec<_> = (0..96)
        .map(|i| record(0x40, 0, 0x9000 + i * 0x1000, 0x1000, &format!("блок{i}"), 0))
        .collect();
    assert_bad(
        parse_partition_table(&table(&records), TABLE_OFFSET, FLASH_SIZE),
        "нет конца",
    );
}

#[test]
fn объединённый_образ_с_пустым_ota_и_сдвигом_таблицы() {
    // Kconfig PARTITION_TABLE_OFFSET допускает назначенное смещение 0x9000.
    let offset = TABLE_OFFSET + 0x1000;
    let t = table(&[block_record(), record(0, 0x10, 0x20000, 0x1000, "ota_0", 0)]);
    let app = block_image();
    let raw = merged(offset, &t, &app);
    let info = parse_flash(&raw, offset).unwrap();
    assert_eq!(info.as_bytes(), raw);
    assert_eq!(info.partition_table().flash_offset, offset);
    assert_eq!(
        info.bootloader(),
        &parse_image(&image_without_hash(), BOOTLOADER_OFFSET).unwrap()
    );
    assert_eq!(
        info.app_image("blok").unwrap(),
        &parse_image(&app, APP_OFFSET).unwrap()
    );
    assert_bad(info.app_image("ota_0"), "стёрт или отсутствует");
    assert_bad(info.app_image("нет"), "нет раздела");
    assert!(parse_flash(&raw, TABLE_OFFSET).is_err());
}

#[test]
fn объединённый_образ_отвергает_порчу_и_выход_за_раздел() {
    let t = table_with_md5();
    let mut raw = merged(TABLE_OFFSET, &t, &block_image());
    raw[APP_OFFSET as usize + 44] ^= 1;
    assert_bad(
        parse_flash(&raw, TABLE_OFFSET),
        "раздел \"blok\" (0x10000): неверный XOR",
    );

    let image = image(IRAM_ENTRY, &[(IRAM_ENTRY, &[0; 0x1000])], None);
    let raw = merged(TABLE_OFFSET, &t, &image);
    assert_bad(parse_flash(&raw, TABLE_OFFSET), "обрезаны данные");

    let raw = merged(TABLE_OFFSET, &t, &[]);
    assert_bad(
        parse_flash(&raw, TABLE_OFFSET),
        "нет записанного приложения",
    );
    let raw = &raw[..TABLE_OFFSET as usize + TABLE_LEN - 1];
    assert_bad(
        parse_flash(raw, TABLE_OFFSET),
        "сектор таблицы разделов обрезан",
    );
}

#[test]
fn хвост_с_дополнительными_данными_явно_отклонён() {
    let mut app = block_image();
    app.push(0x31);
    assert!(matches!(
        parse_image(&app, APP_OFFSET),
        Err(BoardFault::Unsupported(_))
    ));
    let raw = merged(TABLE_OFFSET, &table_with_md5(), &app);
    assert!(
        matches!(parse_flash(&raw, TABLE_OFFSET), Err(BoardFault::Unsupported(s)) if s.contains("раздел"))
    );
}

fn tool_stdout(program: &str, args: &[&str]) -> String {
    let output = Command::new(program).args(args).output().unwrap();
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    String::from_utf8(output.stdout).unwrap()
}

fn compare_image_info(path: &Path, ours: &EspImage<'_>) {
    let report = tool_stdout(
        "esptool",
        &["--chip", "esp32c3", "image-info", path.to_str().unwrap()],
    );
    for expected in [
        format!("Image size: {} bytes", ours.byte_len),
        format!("Entry point: {:#x}", ours.header.entry_addr),
        format!("Segments: {}", ours.header.segment_count),
        format!("Chip ID: {} (ESP32-C3)", ours.header.chip_id),
        format!("Checksum: {:#04x} (valid)", ours.checksum),
        format!("Validation hash: {} (valid)", hex(&ours.sha256.unwrap())),
    ] {
        assert!(
            report.lines().any(|line| line.trim() == expected),
            "{expected}\n{report}"
        );
    }
    for (index, segment) in ours.segments.iter().enumerate() {
        let header_offset = segment.flash_offset - ours.flash_offset - 8; // esp_image_segment_header_t
        let columns = [
            index.to_string(),
            format!("0x{:05x}", segment.data.len()),
            format!("0x{:08x}", segment.load_addr),
            format!("0x{header_offset:08x}"),
        ];
        assert!(
            report.lines().any(|line| line
                .split_whitespace()
                .take(columns.len())
                .eq(columns.iter().map(String::as_str))),
            "{columns:?}\n{report}"
        );
    }
}

#[test]
#[ignore = "требует штатной сборки hello_world, esptool и ESP_IDF_IMAGE_BUILD с путём к build"]
fn штатная_сборка_совпадает_с_esptool_и_генератором_таблиц() {
    // После export.sh: в каталоге сборки выполнить
    // esptool --chip esp32c3 merge-bin -o merged.bin @flash_args.
    let root = std::path::PathBuf::from(std::env::var_os("ESP_IDF_IMAGE_BUILD").unwrap());
    let raw = std::fs::read(root.join("merged.bin")).unwrap();
    let boot_path = root.join("bootloader/bootloader.bin");
    let app_path = root.join("hello_world.bin");
    let table_path = root.join("partition_table/partition-table.bin");

    let flash = parse_flash(&raw, TABLE_OFFSET).unwrap();
    let boot = std::fs::read(&boot_path).unwrap();
    let app = std::fs::read(&app_path).unwrap();
    let table_bytes = std::fs::read(&table_path).unwrap();
    let boot_image = parse_image(&boot, BOOTLOADER_OFFSET).unwrap();
    let app_image = parse_image(&app, APP_OFFSET).unwrap();

    // Штатные параметры hello_world ESP-IDF v6.1.1 из flash_args.
    assert_eq!(flash.bootloader(), &boot_image);
    assert_eq!(flash.app_image("factory").unwrap(), &app_image);
    assert_eq!(
        (
            boot_image.header.spi_mode,
            boot_image.header.spi_speed,
            boot_image.header.flash_size
        ),
        (2, 0xf, FLASH_SIZE)
    );
    assert_eq!(
        flash.partition_table(),
        &parse_partition_table(&table_bytes, TABLE_OFFSET, FLASH_SIZE).unwrap()
    );
    compare_image_info(&boot_path, &boot_image);
    compare_image_info(&app_path, &app_image);

    let idf = std::path::PathBuf::from(std::env::var_os("IDF_PATH").unwrap());
    let script = idf.join("components/partition_table/gen_esp32part.py");
    let csv = tool_stdout(
        "python",
        &[script.to_str().unwrap(), table_path.to_str().unwrap()],
    );
    // Названия и размеры из partition_table/partitions_singleapp.csv ESP-IDF.
    let expected = [
        ("nvs", 1, 2, 0x9000, 0x6000),
        ("phy_init", 1, 1, 0xf000, 0x1000),
        ("factory", 0, 0, APP_OFFSET, 0x100000),
    ];
    assert_eq!(flash.partition_table().partitions.len(), expected.len());
    assert!(flash.partition_table().md5.is_some());
    for (part, expected) in flash.partition_table().partitions.iter().zip(expected) {
        assert_eq!(
            (
                part.label.as_str(),
                part.kind,
                part.subtype,
                part.offset,
                part.size
            ),
            expected
        );
        assert!(
            csv.lines()
                .any(|line| line.starts_with(&format!("{},", part.label))
                    && line.contains(&format!(",{:#x},", part.offset))),
            "{csv}"
        );
    }
}
