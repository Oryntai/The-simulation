//! Стенд чтения flash. Сравнение с транзакциями Espressif QEMU и образами SDK.

pub use sim_boards::{BoardFault, esp_image};

// Включение модуля в шину платы относится к Т2.10.
#[path = "../src/peripheral/esp_flash.rs"]
mod esp_flash;

use esp_flash::{EspFlash, EspFlashReadback};
use esp_image::{DEFAULT_PARTITION_TABLE_OFFSET as TABLE, SegmentKind, parse_flash};

// TRM §3.3, §14.4; ESP-IDF soc/esp32c3/{include/soc/ext_mem_defs.h,register/soc/reg_base.h}.
const DROM: u32 = 0x3c00_0000;
const IROM: u32 = 0x4200_0000;
const SRAM0: u32 = 0x4037_c000;
const SRAM1: u32 = 0x4038_0000;
const PAGE: u32 = 0x1_0000;
const SPI1: u32 = 0x6000_2000;
const SPI0: u32 = 0x6000_3000;
const CACHE: u32 = 0x600c_4000;
const MMU: u32 = 0x600c_5000;
const SRAM_LOCK: u32 = 0x600c_1018;
const SRAM_USAGE: u32 = 0x600c_101c;

// ESP-IDF soc/esp32c3/register/soc/{spi_mem_reg,extmem_reg,sensitive_reg}.h.
const ADDR: u32 = SPI1 + 4;
const USER: u32 = SPI1 + 0x18;
const USER1: u32 = SPI1 + 0x1c;
const USER2: u32 = SPI1 + 0x20;
const RX_BITS: u32 = SPI1 + 0x28;
const STATUS: u32 = SPI1 + 0x2c;
const CS: u32 = SPI1 + 0x34;
const SPI0_CACHE: u32 = SPI0 + 0x3c;
const W0: u32 = SPI1 + 0x58;
const W15: u32 = SPI1 + 0x94;
const CACHE_BUS: u32 = CACHE + 4;
const SYNC: u32 = CACHE + 0x28;
const SYNC_ADDR: u32 = CACHE + 0x2c;
const SYNC_SIZE: u32 = CACHE + 0x30;

const CACHE_BANK: u32 = 0b1110;
const CPU_BANK: u32 = 0b1111;
const MMU_INVALID: u32 = 0x100;
const FLASH_READ: u32 = 1 << 31;
const FLASH_RDID: u32 = 1 << 28;
const FLASH_RDSR: u32 = 1 << 27;
const USR: u32 = 1 << 18;
const COMMAND_MISO: u32 = (1 << 31) | (1 << 28);
const ADDR_PHASE: u32 = 1 << 30;
const DUMMY_PHASE: u32 = 1 << 29;
const MISO_HIGHPART: u32 = 1 << 24;
const ADDR24: u32 = 23 << 26;
const CMD8: u32 = 7 << 28;

// Размер выбран для стенда QEMU. Заголовок синтетических образов объявляет ту же ёмкость.
const CAPACITY: u32 = 2 << 20;
const APP: u32 = 0x10000; // Выравнивание приложения, gen_esp32part.py.
const DATA: u32 = 0x11000; // Смещение тестового раздела из таблицы стенда.
const DATA_LEN: u32 = 0x1000; // Один сектор, gen_esp32part.py.

fn reg(flash: &mut EspFlash, addr: u32) -> Result<Option<u32>, BoardFault> {
    flash.access(SRAM1, addr, 4, None)
}

fn write(flash: &mut EspFlash, addr: u32, val: u32) -> Result<Option<u32>, BoardFault> {
    flash.access(SRAM1, addr, 4, Some(val))
}

fn put(flash: &mut EspFlash, addr: u32, val: u32) {
    assert_eq!(write(flash, addr, val), Ok(Some(0)));
}

fn enable(flash: &mut EspFlash) {
    put(flash, SRAM_USAGE, 0); // ROM при холодном старте в трассе QEMU.
    put(flash, SPI0_CACHE, 1);
    put(flash, CACHE_BUS, 0);
    put(flash, CACHE, 1);
}

#[test]
fn spi_и_окна_совпадают_с_qemu() {
    // QEMU Espressif 9.2.2 (esp_develop_9.2.2_20260417), -M esp32c3,
    // -drive file=flash.bin,if=mtd,format=raw. Проба RV32 из SRAM1 сохраняла
    // результаты LW в 0x3fc81000. Команды SPI задавались SW в порядке ниже.
    // Последний маркер гостя: t0=37 после JALR в 0x42000020.
    let mut raw = vec![0xff; CAPACITY as usize];
    raw[TABLE as usize..TABLE as usize + 8]
        .copy_from_slice(&[0xaa, 0x50, 0, 0, 0x12, 0x34, 0x56, 0x78]);
    // addi t0,x0,37; jalr x0,ra,0 по спецификации RV32I.
    raw[APP as usize + 0x20..APP as usize + 0x28]
        .copy_from_slice(&[0x93, 2, 0x50, 2, 0x67, 0x80, 0, 0]);
    let mut flash = EspFlash::new(&raw, CAPACITY).unwrap();
    let reset_mmu = reg(&mut flash, MMU).unwrap().unwrap();
    put(&mut flash, ADDR, TABLE);
    put(&mut flash, USER1, ADDR24 | 7); // Длина dummy по сбросу, фаза выключена.
    put(&mut flash, RX_BITS, 63); // Восемь байт, длина кодируется числом бит минус один.
    put(&mut flash, SPI1, FLASH_READ);
    let mem0 = reg(&mut flash, W0).unwrap().unwrap();
    let mem1 = reg(&mut flash, W0 + 4).unwrap().unwrap();
    let command_done = reg(&mut flash, SPI1).unwrap().unwrap();
    put(&mut flash, USER, COMMAND_MISO | ADDR_PHASE);
    put(&mut flash, USER2, CMD8 | 3); // CMD_READ из spi_flash_defs.h.
    put(&mut flash, SPI1, USR);
    let usr0 = reg(&mut flash, W0).unwrap().unwrap();
    let usr1 = reg(&mut flash, W0 + 4).unwrap().unwrap();
    put(&mut flash, MMU, 1); // APP / PAGE.
    let mmu1 = reg(&mut flash, MMU).unwrap().unwrap();
    enable(&mut flash);
    let drom = reg(&mut flash, DROM + 0x20).unwrap().unwrap();
    let irom = flash.fetch(IROM + 0x20, 4).unwrap().unwrap();
    put(&mut flash, SYNC, 1);
    let sync_done = reg(&mut flash, SYNC).unwrap().unwrap();
    assert_eq!(
        [
            reset_mmu,
            mem0,
            mem1,
            command_done,
            usr0,
            usr1,
            mmu1,
            drom,
            irom,
            sync_done
        ],
        [
            0x100, 0x50aa, 0x78563412, 0, 0x50aa, 0x78563412, 1, 0x02500293, 0x02500293, 2
        ]
    );
    assert_eq!(flash.fetch(IROM + 0x24, 4), Ok(Some(0x00008067)));
}

#[test]
fn кэш_проверяет_обе_шины_размеры_и_переход_между_страницами() {
    let mut raw = vec![0xff; (PAGE * 3) as usize];
    raw[PAGE as usize - 2..PAGE as usize].copy_from_slice(&[0x11, 0x22]);
    raw[(2 * PAGE) as usize..(2 * PAGE) as usize + 2].copy_from_slice(&[0x33, 0x44]);
    let mut flash = EspFlash::new(&raw, CAPACITY).unwrap();

    put(&mut flash, SRAM_USAGE, CACHE_BANK);
    put(&mut flash, MMU, 0);
    put(&mut flash, MMU + 4, 2);
    assert!(matches!(
        reg(&mut flash, DROM),
        Err(BoardFault::BusError { .. })
    ));
    put(&mut flash, CACHE, 1);
    put(&mut flash, CACHE_BUS, 0);
    assert!(matches!(
        reg(&mut flash, DROM),
        Err(BoardFault::BusError { .. })
    ));
    put(&mut flash, SPI0_CACHE, 1);
    put(&mut flash, CACHE_BUS, 1); // Отключён IBUS.
    assert_eq!(
        flash.access(SRAM1, DROM + PAGE - 1, 1, None),
        Ok(Some(0x22))
    );
    assert!(matches!(
        flash.fetch(IROM, 2),
        Err(BoardFault::BusError { .. })
    ));
    put(&mut flash, CACHE_BUS, 2); // Отключён DBUS.
    assert!(matches!(
        reg(&mut flash, DROM),
        Err(BoardFault::BusError { .. })
    ));
    assert_eq!(flash.fetch(IROM + PAGE - 2, 4), Ok(Some(0x44332211)));
    assert!(matches!(
        flash.fetch(DROM, 2),
        Err(BoardFault::BusError { .. })
    ));
    assert!(matches!(
        flash.access(SRAM1, IROM, 1, None),
        Err(BoardFault::BusError { .. })
    ));
    assert!(matches!(
        reg(&mut flash, IROM + 2),
        Err(BoardFault::Unaligned { .. })
    ));
    assert!(matches!(
        write(&mut flash, DROM, 0),
        Err(BoardFault::WriteToReadOnly { .. })
    ));
    assert!(matches!(
        flash.fetch(IROM + 2 * PAGE, 2),
        Err(BoardFault::BusError { .. })
    ));

    // 128-я запись MMU существует, следующая за ней ячейка — за таблицей.
    put(&mut flash, MMU + 127 * 4, 2);
    assert_eq!(flash.fetch(IROM + 127 * PAGE, 2), Ok(Some(0x4433)));
    assert!(matches!(
        flash.fetch(IROM + 128 * PAGE - 2, 4),
        Err(BoardFault::BusError { .. })
    ));
    assert_eq!(reg(&mut flash, MMU + 128 * 4), Ok(None));
    put(&mut flash, MMU + 8, 0xff);
    assert!(matches!(
        flash.fetch(IROM + 2 * PAGE, 2),
        Err(BoardFault::BadImage(_))
    ));
    assert!(matches!(
        write(&mut flash, MMU + 8, 0x200),
        Err(BoardFault::Unsupported(_))
    ));
    assert_eq!(reg(&mut flash, MMU + 8), Ok(Some(0xff)));
}

#[test]
fn инвалидация_учитывает_диапазон_и_общие_строки_обоих_окон() {
    let mut raw = vec![0; (PAGE * 2) as usize];
    raw[PAGE as usize..].fill(0xa5);
    let mut flash = EspFlash::new(&raw, CAPACITY).unwrap();
    enable(&mut flash);
    put(&mut flash, MMU, 0);
    assert_eq!(reg(&mut flash, DROM), Ok(Some(0)));
    assert_eq!(flash.fetch(IROM + 32, 4), Ok(Some(0)));

    let mut snapshot = flash.clone();
    put(&mut flash, MMU, 1);
    assert!(matches!(
        flash.fetch(IROM, 2),
        Err(BoardFault::Unsupported(_))
    ));
    put(&mut flash, SYNC_ADDR, DROM + 1);
    put(&mut flash, SYNC_SIZE, 1); // Одна 32-байтная строка по TRM §3.3.3.
    put(&mut flash, SYNC, 1);
    assert_eq!(flash.fetch(IROM, 4), Ok(Some(0xa5a5a5a5)));
    assert!(matches!(
        reg(&mut flash, DROM + 32),
        Err(BoardFault::Unsupported(_))
    ));
    put(&mut flash, SYNC_ADDR, 0);
    put(&mut flash, SYNC_SIZE, 0);
    put(&mut flash, SYNC, 3); // RMW при установленном RO-бите DONE.
    assert_eq!(reg(&mut flash, DROM + 32), Ok(Some(0xa5a5a5a5)));
    assert_eq!(reg(&mut snapshot, DROM + 32), Ok(Some(0)));
    put(&mut flash, SYNC_ADDR, SRAM1);
    assert!(matches!(
        write(&mut flash, SYNC, 1),
        Err(BoardFault::Misconfigured(_))
    ));
    put(&mut flash, MMU, MMU_INVALID);
    assert!(matches!(
        reg(&mut flash, DROM),
        Err(BoardFault::BusError { .. })
    ));
}

#[test]
fn sram0_переключается_и_блокируется_с_явным_состоянием_данных() {
    let mut flash = EspFlash::new(&[0xb7; 4], CAPACITY).unwrap();
    assert_eq!(reg(&mut flash, SRAM_USAGE), Ok(Some(CPU_BANK)));
    assert!(matches!(
        flash.fetch(SRAM0, 2),
        Err(BoardFault::Unsupported(_))
    ));
    put(&mut flash, SRAM_USAGE, CACHE_BANK);
    assert!(matches!(
        write(&mut flash, SRAM0, 0x1212),
        Err(BoardFault::BusError { write: true, .. })
    ));

    enable(&mut flash);
    put(&mut flash, MMU, 0);
    assert_eq!(reg(&mut flash, DROM), Ok(Some(0xb7b7b7b7)));
    put(&mut flash, SRAM_USAGE, CPU_BANK);
    assert!(matches!(
        reg(&mut flash, DROM),
        Err(BoardFault::BusError { .. })
    ));
    assert!(matches!(
        flash.fetch(SRAM0, 2),
        Err(BoardFault::Unsupported(_))
    ));
    put(&mut flash, SRAM0, 0x44332211);
    put(&mut flash, SRAM0 + 4, 0x88776655);
    assert_eq!(flash.fetch(SRAM0 + 2, 4), Ok(Some(0x66554433)));
    assert_eq!(flash.fetch(SRAM0 + 2, 2), Ok(Some(0x4433)));
    assert_eq!(reg(&mut flash, SRAM1), Ok(None));

    let mut snapshot = flash.clone();
    put(&mut flash, SRAM_USAGE, CACHE_BANK);
    assert!(matches!(
        flash.fetch(SRAM0, 2),
        Err(BoardFault::BusError { .. })
    ));
    assert!(matches!(
        reg(&mut flash, DROM),
        Err(BoardFault::Unsupported(_))
    ));
    put(&mut flash, SYNC, 1);
    assert_eq!(reg(&mut flash, DROM), Ok(Some(0xb7b7b7b7)));
    put(&mut flash, SRAM_USAGE, CPU_BANK);
    assert!(matches!(
        reg(&mut flash, SRAM0),
        Err(BoardFault::Unsupported(_))
    ));
    put(&mut snapshot, SRAM_LOCK, 1);
    put(&mut snapshot, SRAM_LOCK, 0);
    assert_eq!(reg(&mut snapshot, SRAM_LOCK), Ok(Some(1)));
    assert!(matches!(
        write(&mut snapshot, SRAM_USAGE, CACHE_BANK),
        Err(BoardFault::Misconfigured(_))
    ));
    assert_eq!(reg(&mut snapshot, SRAM0), Ok(Some(0x44332211)));
    snapshot.reset();
    assert_eq!(reg(&mut snapshot, SRAM_USAGE), Ok(Some(CPU_BANK)));
    assert_eq!(reg(&mut snapshot, SRAM_LOCK), Ok(Some(0)));
    assert_eq!(reg(&mut snapshot, MMU), Ok(Some(MMU_INVALID)));
    assert_eq!(reg(&mut snapshot, SYNC), Ok(Some(2)));
    assert_eq!(snapshot.read_physical(0, 4), Ok(&[0xb7; 4][..]));
}

#[test]
fn spi_отказывает_без_профиля_при_неверных_фазах_и_за_пределами() {
    let mut flash = EspFlash::new(&[0x6a; 64], CAPACITY).unwrap();
    assert!(matches!(
        write(&mut flash, SPI1, FLASH_RDID),
        Err(BoardFault::Unsupported(_))
    ));
    assert!(matches!(
        write(&mut flash, SPI1, FLASH_RDSR),
        Err(BoardFault::Unsupported(_))
    ));
    assert!(matches!(
        write(&mut flash, SPI1, 1 << 25),
        Err(BoardFault::Unsupported(_))
    )); // FLASH_PP.
    assert!(matches!(
        flash.access(SRAM1, USER, 2, None),
        Err(BoardFault::UnsupportedRegister { .. })
    ));
    assert!(matches!(
        reg(&mut flash, SPI1 + 0xffc),
        Err(BoardFault::UnsupportedRegister { .. })
    ));
    assert!(matches!(
        reg(&mut flash, USER + 2),
        Err(BoardFault::Unaligned { .. })
    ));
    put(&mut flash, W0, 0x12345678);
    put(&mut flash, ADDR, 63);
    put(&mut flash, RX_BITS, 31);
    assert!(matches!(
        write(&mut flash, SPI1, FLASH_READ),
        Err(BoardFault::Unsupported(_))
    ));
    assert_eq!(reg(&mut flash, W0), Ok(Some(0x12345678)));
    put(&mut flash, RX_BITS, 512); // На один бит шире 16 слов SPI_MEM_W0..W15.
    assert!(matches!(
        write(&mut flash, SPI1, FLASH_READ),
        Err(BoardFault::Misconfigured(_))
    ));
    put(&mut flash, RX_BITS, 511);
    put(&mut flash, ADDR, 0);
    put(&mut flash, SPI1, FLASH_READ);
    assert_eq!(reg(&mut flash, W15), Ok(Some(0x6a6a6a6a)));
    put(&mut flash, CS, 3); // Оба CS запрещены.
    assert!(matches!(
        write(&mut flash, SPI1, FLASH_READ),
        Err(BoardFault::Unsupported(_))
    ));
    put(&mut flash, CS, 2);
    put(&mut flash, USER2, CMD8 | 0x0b); // CMD_FASTRD из spi_flash_defs.h.
    put(&mut flash, USER, COMMAND_MISO | ADDR_PHASE);
    assert!(matches!(
        write(&mut flash, SPI1, USR),
        Err(BoardFault::Misconfigured(_))
    ));
    put(
        &mut flash,
        USER,
        COMMAND_MISO | ADDR_PHASE | DUMMY_PHASE | MISO_HIGHPART,
    );
    put(&mut flash, W0, 0x12345678);
    put(&mut flash, W15, 0x87654321);
    put(&mut flash, RX_BITS, 255);
    put(&mut flash, SPI1, USR);
    assert_eq!(reg(&mut flash, W0), Ok(Some(0x12345678)));
    assert_eq!(reg(&mut flash, W15), Ok(Some(0x6a6a6a6a)));

    // Ответы подставлены стендом, нулевые WIP/WEL требуются по spi_flash_defs.h.
    flash
        .set_readback(EspFlashReadback {
            jedec_id: [0x9e, 0x31, 0x15],
            status1: 0x80,
            status2: None,
        })
        .unwrap();
    assert!(matches!(
        flash.set_readback(EspFlashReadback {
            jedec_id: [0; 3],
            status1: 1,
            status2: None,
        }),
        Err(BoardFault::Misconfigured(_))
    ));
    put(&mut flash, SPI1, FLASH_RDID);
    assert_eq!(reg(&mut flash, W0), Ok(Some(0x1215319e)));
    put(&mut flash, SPI1, FLASH_RDSR);
    assert_eq!(reg(&mut flash, STATUS), Ok(Some(0x80)));
    put(&mut flash, USER, COMMAND_MISO);
    put(&mut flash, RX_BITS, 7);
    put(&mut flash, USER2, CMD8 | 0x35); // CMD_RDSR2.
    assert!(matches!(
        write(&mut flash, SPI1, USR),
        Err(BoardFault::Unsupported(_))
    ));
}

#[test]
fn spi0_и_spi1_учитывают_режим_и_параметры_чтения_независимо() {
    let mut flash = EspFlash::new(b"12345678", CAPACITY).unwrap();
    enable(&mut flash);
    put(&mut flash, MMU, 0);

    // spi_mem_reg.h: SPI_MEM_FREAD_DIO, SPI_MEM_FASTRD_MODE и SPI_MEM_CACHE_FLASH_USR_CMD.
    // spi_flash_defs.h: CMD_FASTRD_DIO=0xbb, 24 бита адреса и четыре dummy-такта.
    let ctrl = (1 << 23) | (1 << 13);
    let user = COMMAND_MISO | ADDR_PHASE | DUMMY_PHASE;
    put(&mut flash, SPI0_CACHE, 5);
    put(&mut flash, SPI0 + 8, ctrl);
    put(&mut flash, SPI0 + 0x18, user);
    put(&mut flash, SPI0 + 0x1c, ADDR24 | 2);
    put(&mut flash, SPI0 + 0x20, CMD8 | 0xbb);
    assert!(matches!(
        reg(&mut flash, DROM),
        Err(BoardFault::Misconfigured(_))
    ));
    put(&mut flash, SPI0 + 0x1c, ADDR24 | 3);
    assert_eq!(
        reg(&mut flash, DROM),
        Ok(Some(u32::from_le_bytes(*b"1234")))
    );
    assert_eq!(reg(&mut flash, W0), Ok(Some(0)));

    put(&mut flash, SPI1 + 8, ctrl);
    put(&mut flash, USER, user);
    put(&mut flash, USER1, ADDR24 | 3);
    put(&mut flash, USER2, CMD8 | 0xbb);
    put(&mut flash, ADDR, 4);
    put(&mut flash, RX_BITS, 31);
    put(&mut flash, SPI1, USR);
    assert_eq!(reg(&mut flash, W0), Ok(Some(u32::from_le_bytes(*b"5678"))));
    assert_eq!(reg(&mut flash, SPI0 + 0x58), Ok(Some(0)));
    put(&mut flash, SPI1 + 8, ctrl & !(1 << 13));
    assert!(matches!(
        write(&mut flash, SPI1, USR),
        Err(BoardFault::Misconfigured(_))
    ));
    assert_eq!(reg(&mut flash, W0), Ok(Some(u32::from_le_bytes(*b"5678"))));
}

fn program(entry: u32, segments: &[(u32, &[u8])]) -> Vec<u8> {
    // esp_app_format.h, esp_image_format.c: 24 байта заголовка, 8 байт
    // заголовка сегмента, XOR от 0xef, выравнивание до 16. DIO, 2 МиБ, C3.
    let mut bytes = vec![0xe9, segments.len() as u8, 2, 0x1f];
    bytes.extend(entry.to_le_bytes());
    bytes.resize(24, 0);
    bytes[12] = 5;
    let mut check = 0xef;
    for &(addr, data) in segments {
        bytes.extend(addr.to_le_bytes());
        bytes.extend((data.len() as u32).to_le_bytes());
        bytes.extend(data);
        check = data.iter().fold(check, |xor, byte| xor ^ byte);
    }
    bytes.resize(bytes.len() / 16 * 16 + 15, 0);
    bytes.push(check);
    bytes
}

fn merged(encrypted: bool) -> Vec<u8> {
    // esp_flash_partitions.h: две записи esp_partition_info_t, 0xff до конца
    // таблицы без MD5. Приложение factory и данные типа NVS.
    let mut flash = program(SRAM1, &[(SRAM1, &[0x13, 0, 0, 0])]); // RV32I NOP.
    flash.resize(TABLE as usize, 0xff);
    for (kind, subtype, offset, label, flags) in [
        (0, 0, APP, "blok", 0u32),
        (1, 2, DATA, "param", u32::from(encrypted)),
    ] {
        let mut record = [0; 32];
        record[..4].copy_from_slice(&[0xaa, 0x50, kind, subtype]);
        record[4..8].copy_from_slice(&offset.to_le_bytes());
        record[8..12].copy_from_slice(&DATA_LEN.to_le_bytes());
        record[12..12 + label.len()].copy_from_slice(label.as_bytes());
        record[28..].copy_from_slice(&flags.to_le_bytes());
        flash.extend(record);
    }
    flash.resize(APP as usize, 0xff);
    // drom начинается после 24+8 байт, irom после ещё 8+8 байт.
    flash.extend(program(
        IROM + 0x30,
        &[(DROM + 0x20, b"abcdWXYZ"), (IROM + 0x30, &[0x13, 0, 0, 0])],
    ));
    flash.resize(DATA as usize, 0xff);
    flash.extend(b"data");
    flash
}

#[test]
fn раздел_и_обе_части_приложения_читаются_из_одних_байтов() {
    let raw = merged(false);
    let image = parse_flash(&raw, TABLE).unwrap();
    let mut flash = EspFlash::from_image(&image).unwrap();
    let app = image.app_image("blok").unwrap();
    assert_eq!(flash.read_partition("param", 1, 3), Ok(&b"ata"[..]));
    assert_eq!(flash.read_partition("param", DATA_LEN, 0), Ok(&[][..]));
    for (label, offset, len) in [
        ("absent", 0, 1),
        ("param", DATA_LEN, 1),
        ("param", 1, usize::MAX),
    ] {
        assert!(matches!(
            flash.read_partition(label, offset, len),
            Err(BoardFault::BadImage(_))
        ));
    }
    assert!(matches!(
        flash.read_partition("param", 3, 2),
        Err(BoardFault::Unsupported(_))
    ));
    assert!(matches!(
        flash.read_physical(u32::MAX, 1),
        Err(BoardFault::BadImage(_))
    ));
    assert!(matches!(
        EspFlash::new(&raw, 1),
        Err(BoardFault::BadImage(_))
    ));
    enable(&mut flash);
    assert!(matches!(
        reg(&mut flash, DROM + 0x20),
        Err(BoardFault::BusError { .. })
    ));
    flash.map_image(app).unwrap();
    assert_eq!(
        reg(&mut flash, DROM + 0x20),
        Ok(Some(u32::from_le_bytes(*b"abcd")))
    );
    assert_eq!(flash.fetch(IROM + 0x30, 4), Ok(Some(0x13)));
    assert_eq!(reg(&mut flash, DROM + 0x30), Ok(Some(0x13)));
    assert_eq!(flash.read_partition("blok", 0x20, 8), Ok(&b"abcdWXYZ"[..]));

    let mut incompatible = app.clone();
    incompatible.segments[1].load_addr += 2;
    assert!(matches!(
        flash.map_image(&incompatible),
        Err(BoardFault::BadImage(_))
    ));
    incompatible.segments[1].load_addr -= 2;
    incompatible.segments[1].data = b"oops";
    assert!(matches!(
        flash.map_image(&incompatible),
        Err(BoardFault::BadImage(_))
    ));
    let mut conflict = app.clone();
    conflict.segments[1].load_addr = IROM;
    conflict.segments[1].flash_offset = 0;
    conflict.segments[1].data = &raw[..4];
    put(&mut flash, MMU, 2);
    assert!(matches!(
        flash.map_image(&conflict),
        Err(BoardFault::BadImage(_))
    ));
    assert_eq!(reg(&mut flash, MMU), Ok(Some(2)));
    let raw = merged(true);
    let encrypted = parse_flash(&raw, TABLE).unwrap();
    let encrypted = EspFlash::from_image(&encrypted).unwrap();
    assert!(matches!(
        encrypted.read_partition("param", 0, 4),
        Err(BoardFault::Unsupported(_))
    ));
}

#[test]
#[ignore = "нужен штатный объединённый образ ESP-IDF v6.1.1 (SIM_T25_FLASH)"]
fn sdk_image_читает_сегменты_штатной_сборки() {
    let raw = std::fs::read(std::env::var_os("SIM_T25_FLASH").expect("путь к flash.bin")).unwrap();
    let image = parse_flash(&raw, TABLE).unwrap();
    let app = image.app_image("factory").unwrap();
    let mut flash = EspFlash::from_image(&image).unwrap();
    flash.map_image(app).unwrap();
    enable(&mut flash);
    let mut seen = (false, false);
    for segment in &app.segments {
        let offset = segment.flash_offset - app.flash_offset;
        assert_eq!(
            flash.read_partition("factory", offset, segment.data.len()),
            Ok(segment.data)
        );
        match segment.kind {
            SegmentKind::Drom | SegmentKind::Irom => {
                for (index, bytes) in segment.data.chunks_exact(4).enumerate() {
                    let addr = segment.load_addr + (index * 4) as u32;
                    let expected = u32::from_le_bytes(bytes.try_into().unwrap());
                    if segment.kind == SegmentKind::Drom {
                        assert_eq!(reg(&mut flash, addr), Ok(Some(expected)));
                        seen.0 = true;
                    } else {
                        assert_eq!(flash.fetch(addr, 4), Ok(Some(expected)));
                        seen.1 = true;
                    }
                }
                println!(
                    "{:?} {:#010x} → {:#08x}, {} байт",
                    segment.kind,
                    segment.load_addr,
                    segment.flash_offset,
                    segment.data.len()
                );
            }
            _ => {}
        }
    }
    assert_eq!(seen, (true, true));
}
