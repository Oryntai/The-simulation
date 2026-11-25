use sim_boards::esp_image::DEFAULT_PARTITION_TABLE_OFFSET as TABLE;
use sim_boards::{BoardFault, BoardId, BoardInput, BoardSim, InputEvent, ResetCause, new_board};

// TRM §3.3.2, начала ROM0 и двух алиасов SRAM1. Загрузчик стенда лежит
// у начала SRAM1, чтобы в его машинном коде хватало одного LUI.
const ROM_BASE: u32 = 0x4000_0000;
const ROM_LEN: usize = 384 * 1024;
const ENTRY: u32 = 0x4038_0000;
const DRAM_ALIAS: u32 = 0x3fc8_0000;

// ESP-IDF soc/esp32c3/register/soc/efuse_reg.h: от RD_WR_DIS до
// RD_SYS_PART2_DATA7 включительно. Поле WAFER_VERSION_MINOR_LO в RD_MAC_SPI_SYS_3.
const EFUSE_WORDS: usize = (0x17c - 0x2c) / size_of::<u32>();
const WAFER_MINOR_WORD: usize = (0x50 - 0x2c) / size_of::<u32>();
const WAFER_MINOR_SHIFT: u32 = 18;

// ESP-IDF esp_app_format.h и gen_esp32part.py, обычный профиль DIO с
// заводским разделом приложения по смещению из partitions_singleapp.csv.
const APP_OFFSET: usize = 0x10000;
const IMAGE_SEGMENT_PAYLOAD: usize = 24 + 8;
const EBREAK: u32 = 0x0010_0073; // RISC-V ISA, EBREAK.

// RV32I, проверочная программа второй ступени. Стек 16-байтного кадра
// (RISC-V psABI §2.1) проверен относительно SP=0x3fcde560, измеренного на
// входе call_start_cpu0 в QEMU Espressif 9.2.2 с загрузчиком ESP-IDF v6.1.1.
// Источники полей: TRM §5.8 GPIO_STRAP_REG, soc/esp32c3/register/soc/efuse_reg.h.
const LOADER: &[u32] = &[
    0xff01_0113, // addi sp,sp,-16
    0x0011_2623, // sw ra,12(sp)
    0x00c1_2283, // lw t0,12(sp)
    0x0212_9e63, // bne t0,ra,ошибка
    0x3fcd_e2b7, // lui t0,0x3fcde
    0x5502_8293, // addi t0,t0,0x550
    0x0251_1863, // bne sp,t0,ошибка
    0x6000_42b7, // lui t0,0x60004
    0x0382_a303, // lw t1,0x38(t0)
    0x0083_7313, // andi t1,t1,8 (GPIO9)
    0x0203_0063, // beqz t1,ошибка
    0x6000_92b7, // lui t0,0x60009
    0x8502_a283, // lw t0,-1968(t0) (0x60008850)
    0x0122_d293, // srli t0,t0,18
    0x0072_f293, // andi t0,t0,7
    0x0030_0313, // li t1,3 (ревизия 0.3 в tools/idf_py_actions/qemu_ext.py)
    0x0062_9463, // bne t0,t1,ошибка
    EBREAK,      // успех
    EBREAK,      // ошибка
];
const SUCCESS: u32 = ENTRY + (LOADER.len() as u32 - 2) * size_of::<u32>() as u32;
const BUDGET: u64 = 128; // В стенде менее 128 инструкций до любого EBREAK.

fn image(entry: u32, segments: &[(u32, &[u32])]) -> Vec<u8> {
    // ESP-IDF esp_app_format.h: magic, число сегментов, DIO, 80 МГц, 2 МиБ.
    let mut bytes = vec![0xe9, segments.len() as u8, 2, 0x1f];
    bytes.extend(entry.to_le_bytes());
    // WP_PIN disabled, SPI drive, CHIP_ID, границы ревизий, reserved, без SHA.
    bytes.extend([0xee, 0, 0, 0, 5, 0, 3, 3, 0, 199, 0, 0, 0, 0, 0, 0]);
    // ESP-IDF esp_image_format.c: XOR сегментов, последний байт блока 16.
    let mut checksum = 0xef;
    for (addr, words) in segments {
        bytes.extend(addr.to_le_bytes());
        bytes.extend((size_of_val(*words) as u32).to_le_bytes());
        for byte in words.iter().flat_map(|word| word.to_le_bytes()) {
            checksum ^= byte;
            bytes.push(byte);
        }
    }
    bytes.resize(bytes.len() / 16 * 16 + 15, 0);
    bytes.push(checksum);
    bytes
}

fn flash(entry: u32, words: &[u32]) -> Vec<u8> {
    flash_segments(entry, &[(entry, words)])
}

fn flash_segments(entry: u32, segments: &[(u32, &[u32])]) -> Vec<u8> {
    // ESP-IDF gen_esp32part.py, PartitionDefinition: magic, app/factory,
    // смещение, размер, 16 байт метки, flags. Остальные записи стёрты.
    let mut table = vec![0xaa, 0x50, 0, 0];
    table.extend((APP_OFFSET as u32).to_le_bytes());
    table.extend((APP_OFFSET as u32).to_le_bytes());
    table.extend(b"factory");
    table.resize(32, 0);
    let mut flash = image(entry, segments);
    flash.resize(TABLE as usize, 0xff);
    flash.extend(table);
    flash.resize(APP_OFFSET, 0xff);
    flash.extend(image(ENTRY, &[(ENTRY, &[EBREAK])]));
    flash
}

fn efuse() -> [u32; EFUSE_WORDS] {
    let mut words = [0; EFUSE_WORDS];
    // QEMU Espressif, readback при профиле ревизии 0.3 из qemu_ext.py.
    words[WAFER_MINOR_WORD] = 3 << WAFER_MINOR_SHIFT;
    words
}

fn rom(words: &[u32], destination: u32, stack: bool) -> Vec<u8> {
    // Синтетический ROM стенда. Программа CPU копирует свой источник через
    // DBUS-алиас SRAM и передаёт управление обычным вызовом RV32I JALR.
    let mut code = Vec::new();
    if stack {
        code.extend([0x3fcd_e137, 0x5601_0113]); // lui sp; addi sp,sp,0x560
    }
    code.push(DRAM_ALIAS | 0x2b7); // lui t0, %hi(DRAM_ALIAS)
    code.push(ROM_BASE | 0x337); // lui t1, %hi(ROM_BASE)
    let source = code.len();
    code.push(0x0003_0313); // addi t1,t1, смещение источника от ROM_BASE
    for i in 0..words.len() {
        let offset = (i * size_of::<u32>()) as u32;
        code.extend([
            0x0003_2383 | offset << 20,                           // lw t2, offset(t1)
            0x0072_a023 | offset >> 5 << 25 | (offset & 31) << 7, // sw t2, offset(t0)
        ]);
    }
    // Адреса стенда находятся в одной странице IRAM, с положительным imm12.
    code.extend([
        (destination & !0xfff) | 0x2b7,            // lui t0, %hi(destination)
        0x0002_80e7 | (destination & 0xfff) << 20, // jalr ra, %lo(destination)(t0)
    ]);
    code[source] |= (code.len() as u32 * 4) << 20;
    code.extend_from_slice(words);
    let mut dump: Vec<u8> = code.into_iter().flat_map(u32::to_le_bytes).collect();
    dump.resize(ROM_LEN, 0);
    dump
}

fn load(flash: &[u8], rom: &[u8], words: &[u32]) -> Box<dyn BoardSim> {
    let mut board = new_board(BoardId::Esp32C3);
    board
        .load_esp_flash(flash, TABLE, rom, words, [true; 3])
        .unwrap();
    board
}

#[test]
fn вторая_ступень_получает_стек_от_rom_и_читает_снимок_платы() {
    let mut board = load(&flash(ENTRY, LOADER), &rom(LOADER, ENTRY, true), &efuse());
    // Два действия независимы: штамп strap остаётся от сброса, текущий GPIO падает.
    board.run_cycles(1).unwrap();
    board.set_pin_input(9, false);
    let mut snapshot = board.clone_state();
    for instance in [&mut board, &mut snapshot] {
        assert_eq!(
            instance.run_cycles(BUDGET),
            Err(BoardFault::Breakpoint { pc: SUCCESS })
        );
    }
    assert_eq!(board.cycles(), snapshot.cycles());
    assert_eq!(board.take_events(), snapshot.take_events());

    board.reset();
    assert_eq!(board.reset_cause(), Some(ResetCause::Host));
    let error = board.run_cycles(1).unwrap_err();
    assert!(matches!(error, BoardFault::Unsupported(text) if text.contains("Joint Download Boot")));
    board.set_pin_input(9, true);
    board.reset();
    assert_eq!(
        board.run_cycles(BUDGET),
        Err(BoardFault::Breakpoint { pc: SUCCESS })
    );

    for on in [false, true] {
        board
            .apply(InputEvent {
                cycle: board.cycles(),
                input: BoardInput::Power { on },
            })
            .unwrap();
    }
    assert_eq!(board.reset_cause(), Some(ResetCause::PowerOn));
    assert_eq!(
        board.run_cycles(BUDGET),
        Err(BoardFault::Breakpoint { pc: SUCCESS })
    );
}

#[test]
fn рукопожатие_отклоняет_другой_вход_и_неверно_скопированный_сегмент() {
    let flash = flash(ENTRY, LOADER);
    let mut board = load(&flash, &rom(LOADER, ENTRY + 4, true), &efuse());
    assert!(matches!(board.run_cycles(BUDGET),
        Err(BoardFault::Unsupported(text)) if text.contains("ожидался вход второй ступени")));

    let mut changed = LOADER.to_vec();
    changed[0] = EBREAK;
    let mut board = load(&flash, &rom(&changed, ENTRY, true), &efuse());
    assert!(matches!(board.run_cycles(BUDGET),
        Err(BoardFault::BadImage(text)) if text.contains("0x3fc80000")));
}

#[test]
fn без_инициализации_sp_доступ_к_стеку_падает() {
    let mut board = load(&flash(ENTRY, LOADER), &rom(LOADER, ENTRY, false), &efuse());
    // Второй оператор стенда пишет ra по sp - 16 + 12 при reset sp=0.
    assert_eq!(
        board.run_cycles(BUDGET),
        Err(BoardFault::BusError {
            pc: ENTRY + 4,
            addr: u32::MAX - 3,
            write: true,
        })
    );
}

#[test]
fn другой_снимок_efuse_доводит_стенд_до_ветки_отказа() {
    let mut words = efuse();
    words[WAFER_MINOR_WORD] = 0;
    let mut board = load(&flash(ENTRY, LOADER), &rom(LOADER, ENTRY, true), &words);
    assert_eq!(
        board.run_cycles(BUDGET),
        Err(BoardFault::Breakpoint { pc: SUCCESS + 4 })
    );
}

#[test]
fn вызов_неизвестного_входа_rom_после_передачи_управления_останавливает_плату() {
    // ESP-IDF esp_rom/esp32c3/ld/esp32c3.rom.ld: ets_get_stack_info = 0x40000054.
    // Синтетический ROM содержит по этому адресу данные, вызов не исполняется.
    let words = [0x4000_02b7, 0x0542_8067]; // lui t0; jalr zero,0x54(t0)
    let mut board = load(&flash(ENTRY, &words), &rom(&words, ENTRY, true), &efuse());
    let error = board.run_cycles(BUDGET).unwrap_err();
    assert!(
        matches!(&error, BoardFault::Unsupported(text) if text.contains("неизвестный вход ROM ESP32-C3 0x40000054"))
    );
    assert_eq!(board.run_cycles(1), Err(error));
}

#[test]
fn неполные_входы_и_несовместимые_режимы_не_заменяются_умолчаниями() {
    let dump = rom(LOADER, ENTRY, true);
    let mut board = load(&flash(ENTRY, LOADER), &dump, &efuse());
    let mut bad_flash = flash(ENTRY, LOADER);
    bad_flash[IMAGE_SEGMENT_PAYLOAD] ^= 1;
    assert!(
        matches!(board.load_esp_flash(&bad_flash, TABLE, &dump, &efuse(), [true; 3]),
        Err(BoardFault::BadImage(text)) if text.contains("XOR"))
    );
    assert!(matches!(board.run_cycles(1), Err(BoardFault::BadImage(_))));

    let flash = flash(ENTRY, LOADER);
    for (rom, words) in [(&[][..], &efuse()[..]), (&dump[..], &[][..])] {
        assert!(matches!(
            board.load_esp_flash(&flash, TABLE, rom, words, [true; 3]),
            Err(BoardFault::Misconfigured(_))
        ));
    }
    // ESP-IDF efuse_reg.h: EFUSE_SECURE_BOOT_EN в RD_REPEAT_DATA2_REG.
    let mut words = efuse();
    words[(0x38 - 0x2c) / size_of::<u32>()] = 1 << 20;
    assert!(
        matches!(board.load_esp_flash(&flash, TABLE, &dump, &words, [true;3]),
        Err(BoardFault::Unsupported(text)) if text.contains("Secure Boot"))
    );

    let mut other = new_board(BoardId::Virtual);
    assert!(matches!(
        other.load_esp_flash(&flash, TABLE, &dump, &efuse(), [true; 3]),
        Err(BoardFault::Unsupported(_))
    ));
}

#[test]
fn вторая_ступень_не_может_занять_стек_rom_или_наложить_алиасы_sram() {
    // ESP-IDF bootloader/subproject/main/ld/esp32c3/bootloader.memory.ld.in, __stack_sentry.
    const STACK_START: u32 = 0x3fcd_c710;
    let rom = rom(LOADER, ENTRY, true);
    let mut board = new_board(BoardId::Esp32C3);
    for (addr, message) in [
        (DRAM_ALIAS, "пересекает другой сегмент"),
        (STACK_START, "вне SRAM загрузчика"),
    ] {
        let flash = flash_segments(ENTRY, &[(ENTRY, LOADER), (addr, &[EBREAK])]);
        assert!(matches!(
            board.load_esp_flash(&flash, TABLE, &rom, &efuse(), [true; 3]),
            Err(BoardFault::BadImage(text)) if text.contains(message)
        ));
    }
}

#[test]
#[ignore = "требуются установленный дамп ROM QEMU, штатный raw flash ESP-IDF и readback eFuse из QEMU"]
fn штатный_дамп_проходит_чтение_rtc_и_останавливается_на_кэше() {
    let read = |key| std::fs::read(std::env::var(key).expect(key)).expect(key);
    let dump = read("SIM_ESP_BOOT_ROM");
    let flash = read("SIM_ESP_BOOT_FLASH");
    let readback = read("SIM_ESP_BOOT_READBACK");
    assert_eq!(readback.len(), EFUSE_WORDS * size_of::<u32>());
    let words: Vec<u32> = readback
        .chunks_exact(size_of::<u32>())
        .map(|word| u32::from_le_bytes(word.try_into().unwrap()))
        .collect();
    let mut board = load(&flash, &dump, &words);
    // QEMU Espressif 9.2.2, -d exec,nochain, one-insn-per-tb=on:
    // индекс 5523 — lw a0,56(a5), RTC_CNTL_RESET_STATE_REG;
    // индекс 5629 — lw a5,4(a4), EXTMEM_ICACHE_CTRL1_REG.
    const RTC_READ: u64 = 5523;
    const PREFIX: u64 = 5629;
    board.run_cycles(RTC_READ).unwrap();
    board.run_cycles(1).unwrap();
    assert_eq!(board.cycles(), RTC_READ + 1);
    board.run_cycles(PREFIX - RTC_READ - 1).unwrap();
    assert_eq!(board.cycles(), PREFIX);
    let stop = BoardFault::UnsupportedRegister {
        pc: 0x4005_113c,
        // ESP-IDF soc/esp32c3/register/soc/extmem_reg.h: EXTMEM_ICACHE_CTRL1_REG.
        addr: 0x600c_4004,
        write: false,
        size: size_of::<u32>() as u8,
        block: "периферии",
    };
    assert_eq!(board.run_cycles(1), Err(stop.clone()));
    assert_eq!(board.cycles(), PREFIX);
    assert_eq!(board.run_cycles(1), Err(stop));
}
