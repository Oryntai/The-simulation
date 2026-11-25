use sim_boards::{BoardFault, BoardId, esp_memory::EspMemory, new_board};

// TRM §3.3.2, таблица 3.3-1.
const ROM0: u32 = 0x4000_0000;
const ROM1: u32 = 0x4004_0000;
const ROM1_DATA: u32 = 0x3ff0_0000;
const SRAM0: u32 = 0x4037_c000;
const SRAM1: u32 = 0x4038_0000;
const SRAM1_DATA: u32 = 0x3fc8_0000;
const RTC_FAST: u32 = 0x5000_0000;

// Даташит v2.5 §3, TRM регистр 5.8 GPIO_STRAP_REG.
const STRAP: u32 = 0x6000_4038;
// ESP-IDF v6.1.1: reg_base.h (DR_REG_EFUSE_BASE), efuse_reg.h (смещения).
const EFUSE: u32 = 0x6000_8800;
const EFUSE_FIRST: u32 = EFUSE + 0x2c;
const EFUSE_RD_DIS: u32 = EFUSE + 0x30;
const EFUSE_KEY0: u32 = EFUSE + 0x9c;
const EFUSE_CHIP_REV: u32 = EFUSE + 0x50;
const EFUSE_LAST: u32 = EFUSE + 0x178;
const EFUSE_CMD: u32 = EFUSE + 0x1d4;

#[test]
fn алиас_rom1_даёт_те_же_байты_и_оба_окна_защищены() {
    let mut memory = EspMemory::default();
    // TRM §3.3.2: от 0x40000000 до 0x4005ffff включительно.
    let mut dump = vec![0; 384 * 1024];
    // Слова, прочитанные через xp /1wx в Espressif QEMU 9.2.2, -M esp32c3.
    for (addr, word) in [
        (ROM0, 0x6910_106fu32),
        (ROM1, 0x0027_ca03),
        (0x4005_fffc, 0x3ff1_be3c),
    ] {
        let offset = (addr - ROM0) as usize;
        dump[offset..offset + 4].copy_from_slice(&word.to_le_bytes());
    }
    memory.install_rom(&dump).unwrap();
    assert_eq!(memory.fetch(ROM0, 4), Ok(0x6910_106f));
    assert_eq!(memory.fetch(ROM1 + 2, 2), Ok(0x0027));
    assert_eq!(memory.read(ROM0, ROM1, 4), Ok(0x0027_ca03));
    assert_eq!(memory.read(ROM0, ROM1_DATA, 4), Ok(0x0027_ca03));
    assert_eq!(memory.read(ROM0, ROM1_DATA + 2, 1), Ok(0x27));
    assert_eq!(memory.read(ROM0, 0x4005_fffc, 4), Ok(0x3ff1_be3c));
    assert_eq!(memory.read(ROM0, 0x3ff1_fffc, 4), Ok(0x3ff1_be3c));
    assert!(matches!(
        memory.fetch(ROM1_DATA, 2),
        Err(BoardFault::BusError { .. })
    ));
    assert!(matches!(
        memory.fetch(0x4005_fffe, 4),
        Err(BoardFault::BusError { .. })
    ));
    assert!(matches!(
        memory.read(ROM0, 0x3ff2_0000, 4),
        Err(BoardFault::BusError { .. })
    ));
    for addr in [ROM0, ROM1, ROM1_DATA] {
        assert_eq!(
            memory.write(SRAM1, addr, 4, u32::MAX),
            Err(BoardFault::WriteToReadOnly { pc: SRAM1, addr })
        );
        assert!(matches!(
            memory.load_ram_segment(addr, &[1, 2, 3, 4]),
            Err(BoardFault::BadImage(_))
        ));
    }
    assert!(matches!(
        memory.install_rom(&dump[..dump.len() - 1]),
        Err(BoardFault::Misconfigured(_))
    ));
    assert_eq!(memory.read(ROM0, ROM1_DATA, 4), Ok(0x0027_ca03));
}

#[test]
fn sram_имеет_общие_алиасы_и_отдельное_окно_rtc() {
    let mut memory = EspMemory::default();
    // Те же ненулевые шаблоны записаны через loader и прочитаны через xp в QEMU.
    for (write_addr, read_addr, word) in [
        (SRAM1_DATA, SRAM1, 0x1234_5678),
        (0x403d_fffc, 0x3fcd_fffc, 0xa5c3_b2e1),
        (SRAM0, SRAM0, 0x55aa_cc33),
        (0x5000_1ffc, 0x5000_1ffc, 0x1122_3344),
    ] {
        memory.write(SRAM1, write_addr, 4, word).unwrap();
        assert_eq!(memory.read(SRAM1, read_addr, 4), Ok(word));
    }
    assert_eq!(memory.fetch(SRAM1, 4), Ok(0x1234_5678));
    let snapshot = memory.clone();
    memory.write(SRAM1, SRAM1_DATA + 1, 1, 0xab).unwrap();
    assert_eq!(memory.fetch(SRAM1, 4), Ok(0x1234_ab78));
    assert_eq!(snapshot.fetch(SRAM1, 4), Ok(0x1234_5678));

    for addr in [
        SRAM1_DATA - 4,
        0x3fce_0000,
        SRAM0 - 4,
        0x403e_0000,
        0x5000_2000,
    ] {
        assert_eq!(
            memory.write(SRAM1, addr, 4, 1),
            Err(BoardFault::BusError {
                pc: SRAM1,
                addr,
                write: true
            })
        );
    }
    assert_eq!(
        memory.write(SRAM1, SRAM1 + 2, 4, 0),
        Err(BoardFault::Unaligned {
            pc: SRAM1,
            addr: SRAM1 + 2,
            size: 4
        })
    );
    // TRM §3.3.1: через IBUS данные читаются только словами.
    assert!(matches!(
        memory.read(SRAM1, SRAM1, 2),
        Err(BoardFault::BusError { .. })
    ));
    assert!(matches!(
        memory.read(SRAM1, RTC_FAST, 4),
        Err(BoardFault::Unsupported(_))
    ));
    assert!(matches!(
        memory.write(SRAM1, RTC_FAST, 0, 0),
        Err(BoardFault::Unsupported(_))
    ));
    assert!(matches!(
        memory.fetch(SRAM1 + 1, 2),
        Err(BoardFault::Unaligned { .. })
    ));
}

#[test]
fn загрузка_учитывает_алиасы_перекрытие_и_целый_диапазон() {
    let mut memory = EspMemory::default();
    // EBREAK из RV32I, little-endian. Загружаем через DBUS, выбираем через IBUS.
    let instruction = 0x0010_0073u32;
    memory
        .load_ram_segment(SRAM1_DATA, &instruction.to_le_bytes())
        .unwrap();
    assert_eq!(memory.fetch(SRAM1, 4), Ok(instruction));
    assert!(matches!(
        memory.load_ram_segment(SRAM1 + 2, &[0; 4]),
        Err(BoardFault::BadImage(_))
    ));
    for addr in [0x3fcd_fffc, 0x403d_fffc, 0x5000_1ffc] {
        assert!(matches!(
            memory.load_ram_segment(addr, &[0; 5]),
            Err(BoardFault::BadImage(_))
        ));
    }
    assert_eq!(memory.fetch(SRAM1, 4), Ok(instruction));
    assert!(matches!(
        memory.read(SRAM1, SRAM1_DATA + 4, 4),
        Err(BoardFault::Unsupported(_))
    ));
}

#[test]
fn регистры_efuse_читают_снимок_и_отклоняют_запись_и_скрытые_данные() {
    let mut memory = EspMemory::default();
    let index = |address| ((address - EFUSE_FIRST) / 4) as usize;
    let mut words = vec![0; index(EFUSE_LAST) + 1];
    // QEMU с default_efuse из ESP-IDF tools/idf_py_actions/qemu_ext.py (C3 rev 0.3).
    words[index(EFUSE_CHIP_REV)] = 0x000c_0000;
    // TRM §4.3.1.2: первый бит RD_DIS закрывает блок 4, KEY0.
    words[index(EFUSE_RD_DIS)] = 1;
    // Отличимый тестовый шаблон последнего слова подтверждает границу окна.
    words[index(EFUSE_LAST)] = 0x89ab_cdef;
    memory.install_efuse_readback(&words).unwrap();
    assert_eq!(memory.read(SRAM1, EFUSE_CHIP_REV, 4), Ok(0x000c_0000));
    assert_eq!(memory.read(SRAM1, EFUSE_RD_DIS, 4), Ok(1));
    assert_eq!(memory.read(SRAM1, EFUSE_KEY0, 4), Ok(0));
    assert_eq!(memory.read(SRAM1, EFUSE_LAST, 4), Ok(0x89ab_cdef));

    words[index(EFUSE_KEY0)] = 1;
    assert!(matches!(
        memory.install_efuse_readback(&words),
        Err(BoardFault::Misconfigured(_))
    ));
    assert!(matches!(
        memory.install_efuse_readback(&words[..words.len() - 1]),
        Err(BoardFault::Misconfigured(_))
    ));
    // EFUSE_PGM_CMD — бит 1 (efuse_reg.h).
    for (addr, value) in [
        (EFUSE, 1),
        (EFUSE_CMD, 2),
        (EFUSE_RD_DIS, 0),
        (EFUSE_LAST, 0),
    ] {
        assert_eq!(
            memory.write(SRAM1, addr, 4, value),
            Err(BoardFault::WriteToReadOnly { pc: SRAM1, addr })
        );
    }
    for (addr, size) in [(EFUSE_LAST + 4, 4), (EFUSE_CHIP_REV, 2)] {
        assert!(matches!(
            memory.read(SRAM1, addr, size),
            Err(BoardFault::UnsupportedRegister { block: "eFuse", .. })
        ));
    }
    // В табл. 3.3-3 TRM указан другой адрес. SDK и QEMU используют 0x60008800.
    assert!(matches!(
        memory.read(SRAM1, 0x6001_a050, 4),
        Err(BoardFault::BusError { .. })
    ));
    assert_eq!(memory.read(SRAM1, EFUSE_RD_DIS, 4), Ok(1));
    assert_eq!(memory.read(SRAM1, EFUSE_LAST, 4), Ok(0x89ab_cdef));
}

#[test]
fn strap_фиксирует_уровни_при_сбросе_и_не_меняется_от_записи_cpu() {
    let mut memory = EspMemory::default();
    // Регистр 5.8 TRM, независимые однобитовые контроли GPIO2, GPIO8, GPIO9.
    for (gpio2, gpio8, gpio9, word) in [
        (true, false, false, 0x01),
        (false, true, false, 0x04),
        (false, false, true, 0x08),
        (true, true, true, 0x0d), // QEMU с gpio.strap_mode=0x0d.
    ] {
        memory.latch_straps(gpio2, gpio8, gpio9);
        assert_eq!(memory.read(SRAM1, STRAP, 4), Ok(word));
    }
    let mut snapshot = memory.clone();
    assert_eq!(
        memory.write(SRAM1, STRAP, 4, 0),
        Err(BoardFault::WriteToReadOnly {
            pc: SRAM1,
            addr: STRAP
        })
    );
    assert_eq!(memory.read(SRAM1, STRAP, 4), Ok(0x0d));
    memory.latch_straps(false, false, false);
    assert_eq!(memory.read(SRAM1, STRAP, 4), Ok(0));
    assert_eq!(snapshot.read(SRAM1, STRAP, 4), Ok(0x0d));
}

#[test]
fn отсутствие_источников_даёт_явную_ошибку() {
    let mut memory = EspMemory::default();
    for addr in [ROM0, ROM1_DATA, EFUSE_FIRST, STRAP] {
        assert!(
            matches!(memory.read(SRAM1, addr, 4), Err(BoardFault::Unsupported(_))),
            "{addr:#x}"
        );
    }
    assert!(matches!(
        memory.fetch(ROM0, 2),
        Err(BoardFault::Unsupported(_))
    ));
}

// Синтетический ELF32 LE, System V ABI. Один PT_LOAD с RV32I по адресу SRAM1.
fn elf(instructions: &[u32]) -> Vec<u8> {
    let mut image = vec![0; 52 + 32];
    let mut put =
        |offset, value: u32| image[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
    put(16, (243 << 16) | 2); // EM_RISCV, ET_EXEC.
    put(20, 1); // EV_CURRENT.
    put(24, SRAM1);
    put(28, 52); // e_phoff.
    put(40, (32 << 16) | 52); // e_phentsize, e_ehsize.
    put(44, 1); // e_phnum.
    put(52, 1); // PT_LOAD.
    put(56, 52 + 32); // p_offset.
    put(60, SRAM1);
    put(64, SRAM1);
    let size = (instructions.len() * 4) as u32;
    put(68, size);
    put(72, size);
    put(76, 5); // PF_R | PF_X.
    put(80, 4); // p_align.
    image[..7].copy_from_slice(b"\x7fELF\x01\x01\x01");
    image.extend(
        instructions
            .iter()
            .flat_map(|instruction| instruction.to_le_bytes()),
    );
    image
}

#[test]
fn boardsim_исполняет_записанное_через_алиас_и_блокирует_запись_в_rom() {
    let mut board = new_board(BoardId::Esp32C3);
    // RV32I: пишем EBREAK в SRAM1 через DBUS по смещению 0x20, прыгаем в её IBUS.
    board
        .load(&elf(&[
            0x3fc8_02b7, // lui t0,0x3fc80
            0x0010_0337, // lui t1,0x100
            0x0733_0313, // addi t1,t1,0x73
            0x0262_a023, // sw t1,32(t0)
            0x4038_03b7, // lui t2,0x40380
            0x0203_8067, // jalr zero,32(t2)
        ]))
        .unwrap();
    assert_eq!(
        board.run_cycles(16),
        Err(BoardFault::Breakpoint { pc: SRAM1 + 0x20 })
    );
    for (lui, addr) in [(0x4000_02b7, ROM0), (0x3ff0_02b7, ROM1_DATA)] {
        // RV32I: sw zero,0(t0) и недостижимая EBREAK.
        board.load(&elf(&[lui, 0x0002_a023, 0x0010_0073])).unwrap();
        assert_eq!(
            board.run_cycles(16),
            Err(BoardFault::WriteToReadOnly {
                pc: SRAM1 + 4,
                addr
            })
        );
    }
    // RV32I: lw t1,0x30(t0) обращается к eFuse без снимка, даже на пути старого ELF.
    board
        .load(&elf(&[0x6000_92b7, 0x8302_a303, 0x0010_0073]))
        .unwrap();
    assert!(
        matches!(board.run_cycles(16), Err(BoardFault::Unsupported(reason)) if reason.contains("eFuse"))
    );
}
