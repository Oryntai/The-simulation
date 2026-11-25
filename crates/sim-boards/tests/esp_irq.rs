//! Потактовые проверки trap и прерываний ESP32-C3.

use sim_boards::{BoardFault, BoardId, new_board};

#[test]
#[ignore = "нужны ROM QEMU Espressif, raw flash ESP-IDF v6.1.1 и снимок eFuse"]
fn штатная_rom_проходит_настройку_csr() {
    let read = |key| std::fs::read(std::env::var(key).expect(key)).expect(key);
    let rom = read("SIM_ESP_BOOT_ROM");
    let flash = read("SIM_ESP_BOOT_FLASH");
    let efuse = read("SIM_ESP_BOOT_READBACK");
    // ESP-IDF, efuse_reg.h: RD_WR_DIS..=RD_SYS_PART2_DATA7.
    assert_eq!(efuse.len(), 0x17c - 0x2c);
    let words = efuse
        .chunks_exact(size_of::<u32>())
        .map(|word| u32::from_le_bytes(word.try_into().unwrap()))
        .collect::<Vec<_>>();
    let mut board = new_board(BoardId::Esp32C3);
    board
        .load_esp_flash(
            &flash,
            sim_boards::esp_image::DEFAULT_PARTITION_TABLE_OFFSET,
            &rom,
            &words,
            [true; 3],
        )
        .unwrap();
    // QEMU 9.2.2, потактовая трасса до чтения RTC_CNTL_RESET_STATE_REG.
    board.run_cycles(5523).unwrap();
    assert_eq!(board.cycles(), 5523);
    assert!(matches!(
        board.run_cycles(1),
        Err(BoardFault::UnsupportedRegister {
            pc: 0x4004_8a94,
            addr: 0x6000_8038, // ESP-IDF esp32c3/register/soc/rtc_cntl_reg.h.
            write: false,
            size: 4,
            ..
        })
    ));
}

// TRM §3.3.2, начало SRAM1 на IBUS. Вектор на следующей границе 256 байт (§1.4.2).
const ENTRY: u32 = 0x4038_0000;
const VECTOR: u32 = ENTRY + 0x100;
// ESP-IDF soc/esp32c3/register/soc/{interrupt_core0_reg,reg_base,system_reg}.h.
const MATRIX: u32 = 0x600c_2000;
const SOFTWARE: u32 = 0x600c_0028;
const ENABLE: u32 = 0x104;
const CLEAR: u32 = 0x10c;
const PENDING: u32 = 0x110;
const PRIORITY: u32 = 0x114;
const THRESHOLD: u32 = 0x194;
const SOFTWARE_MAP: u32 = 50 * 4;

// Регистры стенда по RISC-V psABI: t0–t2 и s0–s3 принадлежат общей схеме опыта.
const T0: u32 = 5;
const T1: u32 = 6;
const T2: u32 = 7;
const S0: u32 = 8;
const S1: u32 = 9;
const S2: u32 = 18;
const S3: u32 = 19;
// RISC-V ISA: SYSTEM и машинные CSR, C для смещения IALIGN=16.
const EBREAK: u32 = 0x0010_0073;
const ECALL: u32 = 0x0000_0073;
const MRET: u32 = 0x3020_0073;
const WFI: u32 = 0x1050_0073;
const FENCE: u32 = 0x0ff0_000f;
const NOP: u32 = 0x0000_0013;
const MSTATUS: u32 = 0x300;
const MTVEC: u32 = 0x305;
const MSCRATCH: u32 = 0x340;
const MEPC: u32 = 0x341;
const MCAUSE: u32 = 0x342;
const MTVAL: u32 = 0x343;

#[derive(Default)]
struct Program(Vec<u32>);

impl Program {
    fn pc(&self) -> u32 {
        ENTRY + self.0.len() as u32 * 4
    }

    fn emit(&mut self, instruction: u32) {
        self.0.push(instruction);
    }

    fn place(&mut self, pc: u32) {
        assert!(pc >= self.pc());
        self.0.resize(((pc - ENTRY) / 4) as usize, EBREAK);
    }

    fn li(&mut self, rd: u32, value: u32) {
        // RV32I LUI + ADDI, младший immediate знаковый.
        let low = value & 0xfff;
        let upper = value.wrapping_add(0x800) & !0xfff;
        self.emit(upper | rd << 7 | 0x37);
        self.emit(low << 20 | rd << 15 | rd << 7 | 0x13);
    }

    fn csr(&mut self, funct: u32, rd: u32, addr: u32, src: u32) {
        // Unprivileged ISA Zicsr: csr[31:20], rs1/uimm[19:15], funct3[14:12].
        self.emit(addr << 20 | src << 15 | funct << 12 | rd << 7 | 0x73);
    }

    fn lw(&mut self, rd: u32, base: u32, offset: u32) {
        self.emit(offset << 20 | base << 15 | 2 << 12 | rd << 7 | 0x03);
    }

    fn sw(&mut self, src: u32, base: u32, offset: u32) {
        self.emit(
            (offset >> 5) << 25 | src << 20 | base << 15 | 2 << 12 | (offset & 31) << 7 | 0x23,
        );
    }

    fn write(&mut self, base: u32, offset: u32, value: u32) {
        self.li(T0, value);
        self.sw(T0, base, offset);
    }

    fn assert_reg(&mut self, actual: u32, expected: u32) {
        assert_ne!(actual, T1);
        self.li(T1, expected);
        // beq actual,t1,+8; ebreak. Unprivileged ISA, B-type immediate.
        self.emit(T1 << 20 | actual << 15 | 4 << 8 | 0x63);
        self.emit(EBREAK);
    }

    fn assert_csr(&mut self, addr: u32, expected: u32) {
        self.csr(2, T2, addr, 0);
        self.assert_reg(T2, expected);
    }

    fn elf(&self) -> Vec<u8> {
        // ELF32 System V ABI и RISC-V psABI: ET_EXEC, EM_RISCV, один RX PT_LOAD.
        let header = 52u16;
        let phdr = 32u16;
        let size = self.0.len() as u32 * 4;
        let mut out = vec![0x7f, b'E', b'L', b'F', 1, 1, 1, 0];
        out.resize(16, 0);
        for v in [2u16, 243] {
            out.extend(v.to_le_bytes());
        }
        for v in [1, ENTRY, u32::from(header), 0, 0] {
            out.extend(v.to_le_bytes());
        }
        for v in [header, phdr, 1, 0, 0, 0] {
            out.extend(v.to_le_bytes());
        }
        for v in [1, u32::from(header + phdr), ENTRY, ENTRY, size, size, 5, 4] {
            out.extend(v.to_le_bytes());
        }
        out.extend(self.0.iter().flat_map(|word| word.to_le_bytes()));
        out
    }

    fn run(&self, budget: u64) -> (Result<sim_boards::Advance, BoardFault>, u64) {
        let mut board = new_board(BoardId::Esp32C3);
        board.load(&self.elf()).unwrap();
        let result = board.run_cycles(budget);
        (result, board.cycles())
    }
}

#[test]
fn csr_zicsr_и_ecall_с_возвратом() {
    let mut p = Program::default();
    p.li(S0, VECTOR | 1);
    p.csr(1, 0, MTVEC, S0);
    p.csr(5, T0, MSCRATCH, 3);
    p.assert_reg(T0, 0);
    p.csr(6, T0, MSCRATCH, 4);
    p.assert_reg(T0, 3);
    p.csr(7, T0, MSCRATCH, 1);
    p.assert_reg(T0, 7);
    p.assert_csr(MSCRATCH, 6);
    p.csr(6, 0, MSTATUS, 8);
    let ecall = p.pc();
    p.emit(ECALL);
    p.assert_csr(MSTATUS, 0x88); // Privileged ISA: после MRET MPP=U, MPIE=MIE=1.
    let done = p.pc();
    p.emit(EBREAK);
    p.place(VECTOR); // Синхронное исключение идёт в слот 0 даже в режиме vectored.
    p.assert_csr(MCAUSE, 11); // Privileged ISA, ECALL из M-mode.
    p.assert_csr(MEPC, ecall);
    p.assert_csr(MTVAL, 0);
    p.assert_csr(MSTATUS, 0x1880); // MPP=M, MPIE=1, MIE=0.
    p.li(T0, ecall + 4);
    p.csr(1, 0, MEPC, T0);
    p.emit(MRET);
    assert_eq!(p.run(128).0, Err(BoardFault::Breakpoint { pc: done }));
}

#[test]
fn неизвестный_csr_останавливает_даже_с_обработчиком() {
    let mut p = Program::default();
    p.li(T0, VECTOR | 1);
    p.csr(1, 0, MTVEC, T0);
    let pc = p.pc();
    // Адрес CUSTOM RW из таблицы Privileged ISA, отсутствует в TRM §1.4.1.
    p.csr(2, T0, 0x7ff, 0);
    p.place(VECTOR);
    p.emit(EBREAK);
    assert_eq!(
        p.run(32),
        (
            Err(BoardFault::UnsupportedRegister {
                pc,
                addr: 0x7ff,
                write: false,
                size: 4,
                block: "CSR RV32",
            }),
            3
        )
    );
}

#[test]
fn недоступный_вектор_не_зацикливает_исключение() {
    let mut p = Program::default();
    // TRM §3.3.2, диапазон вне памяти инструкций C3.
    p.li(T0, 0x1000_0000 | 1);
    p.csr(1, 0, MTVEC, T0);
    let expected_pc = p.pc();
    p.emit(ECALL);
    assert!(matches!(
        p.run(64),
        (Err(BoardFault::Lockup { pc, why }), 3)
            if pc == expected_pc && why.contains("0x10000000")
    ));
}

fn software_program(low: u32, high: u32, check_mmio: bool) -> (Program, u32) {
    let mut p = Program::default();
    // Номера/приоритеты двух линий выбраны из диапазонов TRM §1.5.
    let mask = (1 << low) | (1 << high);
    p.li(S0, MATRIX);
    p.li(S1, SOFTWARE);
    p.li(T0, VECTOR | 1);
    p.csr(1, 0, MTVEC, T0);
    p.write(S0, SOFTWARE_MAP, low);
    p.write(S0, SOFTWARE_MAP + 4, high);
    p.write(S0, PRIORITY + low * 4, 2);
    p.write(S0, PRIORITY + high * 4, 3);
    p.write(S0, ENABLE, mask);
    p.write(S0, THRESHOLD, 4);
    p.write(S1, 0, 1);
    p.write(S1, 4, 1);
    if check_mmio {
        p.lw(T0, S0, PENDING);
        p.assert_reg(T0, 0);
        // TRM §8.3.4, номера 50 и 51 в верхнем регистре источников.
        p.lw(T0, S0, 0xfc);
        p.assert_reg(T0, (1 << (50 - 32)) | (1 << (51 - 32)));
    }
    p.write(S0, THRESHOLD, 2);
    p.emit(FENCE);
    if check_mmio {
        p.lw(T0, S0, PENDING);
        p.assert_reg(T0, mask);
    }
    p.csr(6, 0, MSTATUS, 8);
    let interrupted = p.pc();
    p.assert_reg(S2, 1);
    p.assert_reg(S3, 1);
    let done = p.pc();
    p.emit(EBREAK);

    // Каждый вектор содержит JAL x0 к своему обработчику. Immediate RV32I.
    let mut vectors = [(low, VECTOR + 0x100), (high, VECTOR + 0x200)];
    vectors.sort();
    for (line, handler) in vectors {
        let slot = VECTOR + line * 4;
        p.place(slot);
        let delta = handler - slot;
        assert!(delta < 1 << 11);
        p.emit(delta << 20 | 0x6f);
    }

    p.place(VECTOR + 0x100);
    p.assert_csr(MCAUSE, 0x8000_0000 | low);
    p.assert_csr(MEPC, interrupted);
    p.assert_reg(S3, 1); // Высокий приоритет обслужен первым.
    p.write(S1, 0, 0);
    p.li(S2, 1);
    p.emit(MRET);

    p.place(VECTOR + 0x200);
    p.assert_csr(MCAUSE, 0x8000_0000 | high);
    p.assert_csr(MEPC, interrupted);
    p.assert_reg(S2, 0);
    if check_mmio {
        p.write(S0, CLEAR, 1 << high);
        p.write(S0, CLEAR, 0);
        p.lw(T0, S0, PENDING);
        p.assert_reg(T0, mask); // CLEAR не сбрасывает уровень источника.
    }
    p.write(S1, 4, 0);
    p.li(S3, 1);
    p.emit(MRET);
    (p, done)
}

#[test]
fn freertos_irq_приоритет_маска_подтверждение_и_возврат() {
    let (p, done) = software_program(5, 7, true);
    assert_eq!(p.run(256).0, Err(BoardFault::Breakpoint { pc: done }));
}

#[test]
fn wfi_учитывает_локальную_маску_независимо_от_mie() {
    for enabled in [false, true] {
        let mut p = Program::default();
        p.li(S0, MATRIX);
        p.li(S1, SOFTWARE);
        p.write(S0, SOFTWARE_MAP, 5);
        p.write(S0, PRIORITY + 5 * 4, 1);
        p.write(S0, ENABLE, if enabled { 1 << 5 } else { 0 });
        p.write(S1, 0, 1);
        p.emit(WFI);
        let done = p.pc();
        p.emit(EBREAK);
        if enabled {
            assert_eq!(p.run(64).0, Err(BoardFault::Breakpoint { pc: done }));
        } else {
            let (result, cycles) = p.run(64);
            assert!(result.is_ok());
            assert_eq!(cycles, 64);
        }
    }
}

#[test]
fn pmp_tor_запрещает_запись_и_сохраняет_адрес_ловушки() {
    let mut p = Program::default();
    // TRM §3.3.2, SRAM1 DBUS; отдельное слово данных на границе TOR.
    const DATA: u32 = 0x3fc8_1000;
    p.li(S0, DATA);
    p.write(S0, 0, 42);
    p.li(T0, VECTOR | 1);
    p.csr(1, 0, MTVEC, T0);
    // TRM §1.8, ESP-IDF riscv/csr.h: нижняя и верхняя границы >>2,
    // cfg[15:8]=L|TOR|R. Запись pmpaddr0 затем блокируется соседним TOR.
    p.li(T0, DATA >> 2);
    p.csr(1, 0, 0x3b0, T0);
    p.li(T0, (DATA + 4) >> 2);
    p.csr(1, 0, 0x3b1, T0);
    p.li(T0, 0x8900);
    p.csr(1, 0, 0x3a0, T0);
    p.csr(1, 0, 0x3b0, 0);
    p.assert_csr(0x3b0, DATA >> 2);
    p.csr(1, 0, 0x3a0, 0);
    p.assert_csr(0x3a0, 0x8900);
    let fault = p.pc();
    p.sw(0, S0, 0);
    p.lw(T0, S0, 0);
    p.assert_reg(T0, 42);
    let done = p.pc();
    p.emit(EBREAK);
    p.place(VECTOR);
    p.assert_csr(MCAUSE, 7); // Privileged ISA, store access fault.
    p.assert_csr(MEPC, fault);
    p.assert_csr(MTVAL, DATA);
    p.li(T0, fault + 4);
    p.csr(1, 0, MEPC, T0);
    p.emit(MRET);
    assert_eq!(p.run(128).0, Err(BoardFault::Breakpoint { pc: done }));
}

#[test]
fn счётчик_циклов_учитывает_запись_и_разрешение() {
    let mut p = Program::default();
    // ROM включает счётчик через 0x800/0x801. ESP-IDF riscv/rv_utils.h
    // читает 0x7e2, TRM рег. 1.12–1.14 задают биты CYCLE, COUNT_EN.
    p.csr(5, 0, 0x800, 1);
    p.csr(5, 0, 0x801, 1);
    p.csr(5, 0, 0x7e2, 0);
    p.emit(NOP);
    p.csr(2, T0, 0x7e2, 0);
    p.assert_reg(T0, 2);
    p.csr(5, 0, 0x7e1, 0);
    p.csr(2, T0, 0x7e2, 0);
    p.emit(NOP);
    p.csr(2, T2, 0x802, 0);
    p.assert_reg(T0, 6);
    p.assert_reg(T2, 6);
    let done = p.pc();
    p.emit(EBREAK);
    assert_eq!(p.run(64).0, Err(BoardFault::Breakpoint { pc: done }));
}

#[test]
fn программные_irq_совпадают_с_векторами_и_возвратом_qemu() {
    // QEMU Espressif 9.2.2, -M esp32c3, -device loader, one-insn-per-tb=on.
    // В трассе сначала 0x40380114, затем 0x4038011c и финальный PC 0x403800b0.
    // Для этой сверки порядок номеров совпадает с порядком приоритетов.
    let (p, done) = software_program(7, 5, false);
    assert_eq!(done, 0x4038_00b0);
    // QEMU перед остановом исполнил 78 инструкций. Модель отводит
    // ещё два такта на два входа в IRQ.
    assert_eq!(
        p.run(256),
        (Err(BoardFault::Breakpoint { pc: done }), 78 + 2)
    );
}
