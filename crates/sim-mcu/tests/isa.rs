//! Покомандные тесты ядра, написанные по спецификации RISC-V (Volume I 20191213,
//! Volume II 20211203): кодировки собираются функциями-кодировщиками по таблицам
//! форматов R/I/S/B/U/J (гл. 2.2–2.3), а не нашим декодером.

use sim_mcu::bus::{FLASH_BASE, RAM_BASE};
use sim_mcu::cpu::{Op, cause, decode};
use sim_mcu::{FaultKind, FaultSource, HaltReason, Mcu, McuError};

// ---------- кодировщики по таблицам форматов ----------

fn r(f7: u32, rs2: u32, rs1: u32, f3: u32, rd: u32, op: u32) -> u32 {
    (f7 << 25) | (rs2 << 20) | (rs1 << 15) | (f3 << 12) | (rd << 7) | op
}
fn i(imm: i32, rs1: u32, f3: u32, rd: u32, op: u32) -> u32 {
    (((imm as u32) & 0xfff) << 20) | (rs1 << 15) | (f3 << 12) | (rd << 7) | op
}
fn s(imm: i32, rs2: u32, rs1: u32, f3: u32) -> u32 {
    let m = imm as u32;
    (((m >> 5) & 0x7f) << 25) | (rs2 << 20) | (rs1 << 15) | (f3 << 12) | ((m & 0x1f) << 7) | 0x23
}
fn b(imm: i32, rs2: u32, rs1: u32, f3: u32) -> u32 {
    let m = imm as u32;
    (((m >> 12) & 1) << 31)
        | (((m >> 5) & 0x3f) << 25)
        | (rs2 << 20)
        | (rs1 << 15)
        | (f3 << 12)
        | (((m >> 1) & 0xf) << 8)
        | (((m >> 11) & 1) << 7)
        | 0x63
}
fn lui(rd: u32, imm20: u32) -> u32 {
    (imm20 << 12) | (rd << 7) | 0x37
}
fn jal(rd: u32, imm: i32) -> u32 {
    let m = imm as u32;
    (((m >> 20) & 1) << 31)
        | (((m >> 1) & 0x3ff) << 21)
        | (((m >> 11) & 1) << 20)
        | (((m >> 12) & 0xff) << 12)
        | (rd << 7)
        | 0x6f
}
fn addi(rd: u32, rs1: u32, imm: i32) -> u32 {
    i(imm, rs1, 0, rd, 0x13)
}
fn jalr(rd: u32, rs1: u32, imm: i32) -> u32 {
    i(imm, rs1, 0, rd, 0x67)
}
fn lw(rd: u32, rs1: u32, imm: i32) -> u32 {
    i(imm, rs1, 2, rd, 0x03)
}
fn sw(rs2: u32, rs1: u32, imm: i32) -> u32 {
    s(imm, rs2, rs1, 2)
}
fn csrrw(rd: u32, csr: u32, rs1: u32) -> u32 {
    i(csr as i32, rs1, 1, rd, 0x73)
}
fn csrrs(rd: u32, csr: u32, rs1: u32) -> u32 {
    i(csr as i32, rs1, 2, rd, 0x73)
}
const NOP: u32 = 0x0000_0013;
const ECALL: u32 = 0x0000_0073;
const EBREAK: u32 = 0x0010_0073;
const MRET: u32 = 0x3020_0073;
const WFI: u32 = 0x1050_0073;
const MSTATUS: u32 = 0x300;
const MIE: u32 = 0x304;
const MTVEC: u32 = 0x305;
const MEPC: u32 = 0x341;
const MCAUSE: u32 = 0x342;
const MTVAL: u32 = 0x343;

/// Загрузка 32-битной константы (lui + addi с поправкой на знак младших 12 бит).
fn li(rd: u32, v: u32) -> [u32; 2] {
    let hi = v.wrapping_add(0x800) >> 12;
    let lo = v.wrapping_sub(hi << 12) as i32;
    [lui(rd, hi & 0xfffff), addi(rd, rd, lo)]
}

/// Программа во флеше.
#[derive(Default)]
struct Prog(Vec<u8>);

impl Prog {
    fn w(&mut self, x: u32) -> &mut Self {
        self.0.extend_from_slice(&x.to_le_bytes());
        self
    }
    fn ws(&mut self, xs: &[u32]) -> &mut Self {
        for &x in xs {
            self.w(x);
        }
        self
    }
    fn h(&mut self, x: u16) -> &mut Self {
        self.0.extend_from_slice(&x.to_le_bytes());
        self
    }
    /// Дополнить nop до байтового смещения.
    fn at(&mut self, off: usize) -> &mut Self {
        while self.0.len() < off {
            self.w(NOP);
        }
        self
    }
    fn mcu(&self) -> Mcu {
        Mcu::from_image(&self.0, FLASH_BASE).expect("образ")
    }
}

/// Исполнять до остановки (не дольше `max` тактов).
fn run(m: &mut Mcu, max: u64) -> HaltReason {
    m.run_cycles(max);
    m.halt_reason().cloned().expect("ядро не остановилось")
}

/// Пролог: mtvec ← FLASH_BASE + off (обработчик), t0 = x5 — рабочий.
fn set_mtvec(p: &mut Prog, off: u32, mode: u32) {
    p.ws(&li(5, FLASH_BASE + off + mode)).w(csrrw(0, MTVEC, 5));
}

#[test]
fn x0_is_hardwired_to_zero() {
    let mut p = Prog::default();
    p.w(addi(0, 0, 5)) // запись в x0 игнорируется
        .w(lui(0, 0x12345))
        .w(addi(1, 0, 0)) // x1 = x0
        .w(r(0, 0, 0, 0, 2, 0x33)) // add x2, x0, x0
        .h(0x4015) // c.li x0, 5 (HINT)
        .w(addi(3, 0, 7))
        .w(EBREAK);
    let mut m = p.mcu();
    run(&mut m, 1000);
    assert_eq!(m.reg(0), 0);
    assert_eq!(m.reg(1), 0);
    assert_eq!(m.reg(2), 0);
    assert_eq!(m.reg(3), 7);
}

#[test]
fn immediates_are_sign_extended() {
    let mut p = Prog::default();
    p.w(addi(1, 0, -1))
        .w(lui(2, 0x80000))
        .w(i(-1, 0, 2, 3, 0x13)) // slti x3, x0, -1 → 0 (0 < −1 ложно)
        .w(i(-1, 0, 3, 4, 0x13)) // sltiu x4, x0, -1 → 1 (0 < 0xFFFFFFFF)
        .w(addi(5, 0, -2048))
        .w(i(-1, 2, 4, 6, 0x13)) // xori x6, x2, -1 → !0x8000_0000
        .w(EBREAK);
    let mut m = p.mcu();
    run(&mut m, 1000);
    assert_eq!(m.reg(1), 0xffff_ffff);
    assert_eq!(m.reg(2), 0x8000_0000);
    assert_eq!(m.reg(3), 0);
    assert_eq!(m.reg(4), 1);
    assert_eq!(m.reg(5), (-2048i32) as u32);
    assert_eq!(m.reg(6), 0x7fff_ffff);
}

#[test]
fn loads_sign_and_zero_extend() {
    let mut p = Prog::default();
    p.ws(&li(10, RAM_BASE))
        .ws(&li(11, 0x8001_80fe))
        .w(sw(11, 10, 0))
        .w(i(0, 10, 0, 1, 0x03)) // lb x1 ← 0xfe
        .w(i(0, 10, 4, 2, 0x03)) // lbu x2
        .w(i(0, 10, 1, 3, 0x03)) // lh x3 ← 0x80fe
        .w(i(0, 10, 5, 4, 0x03)) // lhu x4
        .w(i(2, 10, 1, 5, 0x03)) // lh x5 ← 0x8001
        .w(i(3, 10, 0, 6, 0x03)) // lb x6 ← 0x80
        .w(s(1, 0, 10, 0)) // sb x0 → байт 1
        .w(lw(7, 10, 0))
        .w(EBREAK);
    let mut m = p.mcu();
    run(&mut m, 1000);
    assert_eq!(m.reg(1), 0xffff_fffe);
    assert_eq!(m.reg(2), 0xfe);
    assert_eq!(m.reg(3), 0xffff_80fe);
    assert_eq!(m.reg(4), 0x80fe);
    assert_eq!(m.reg(5), 0xffff_8001);
    assert_eq!(m.reg(6), 0xffff_ff80);
    assert_eq!(m.reg(7), 0x8001_00fe);
}

#[test]
fn shifts_use_low_five_bits() {
    let mut p = Prog::default();
    p.w(addi(1, 0, 1))
        .w(addi(2, 0, 33))
        .w(r(0, 2, 1, 1, 3, 0x33)) // sll x3, x1, x2 → 1 << 1
        .ws(&li(4, 0x8000_0000))
        .w(r(0x20, 2, 4, 5, 5, 0x33)) // sra x5, x4, 33 → сдвиг на 1 с размножением знака
        .w(r(0, 2, 4, 5, 6, 0x33)) // srl
        .w(EBREAK);
    let mut m = p.mcu();
    run(&mut m, 1000);
    assert_eq!(m.reg(3), 2);
    assert_eq!(m.reg(5), 0xc000_0000);
    assert_eq!(m.reg(6), 0x4000_0000);
}

#[test]
fn division_edge_cases_follow_table_7_1() {
    let mut p = Prog::default();
    p.w(addi(1, 0, 7))
        .ws(&li(2, 0x8000_0000))
        .w(addi(3, 0, -1))
        .w(r(1, 0, 1, 4, 10, 0x33)) // div 7/0 → −1
        .w(r(1, 0, 1, 5, 11, 0x33)) // divu 7/0 → 2³²−1
        .w(r(1, 0, 1, 6, 12, 0x33)) // rem 7%0 → 7
        .w(r(1, 0, 1, 7, 13, 0x33)) // remu 7%0 → 7
        .w(r(1, 3, 2, 4, 14, 0x33)) // div INT_MIN/−1 → INT_MIN
        .w(r(1, 3, 2, 6, 15, 0x33)) // rem INT_MIN%−1 → 0
        .w(r(1, 2, 2, 1, 16, 0x33)) // mulh INT_MIN·INT_MIN → 2^30
        .w(r(1, 3, 3, 2, 17, 0x33)) // mulhsu −1·0xFFFFFFFF → −1
        .w(r(1, 3, 3, 3, 18, 0x33)) // mulhu → 0xFFFFFFFE
        .w(EBREAK);
    let mut m = p.mcu();
    run(&mut m, 2000);
    assert_eq!(m.reg(10), u32::MAX);
    assert_eq!(m.reg(11), u32::MAX);
    assert_eq!(m.reg(12), 7);
    assert_eq!(m.reg(13), 7);
    assert_eq!(m.reg(14), 0x8000_0000);
    assert_eq!(m.reg(15), 0);
    assert_eq!(m.reg(16), 0x4000_0000);
    assert_eq!(m.reg(17), u32::MAX);
    assert_eq!(m.reg(18), 0xffff_fffe);
}

#[test]
fn branches_and_jumps_link_correctly() {
    let mut p = Prog::default();
    // 0: addi x1, x0, 1; 4: beq x1, x0, +8 (не берётся); 8: bne x1, x0, +8 → 16
    // 12: addi x2, x0, 99 (пропускается); 16: jal x3, +8 → 24; 20: addi x4, x0, 99
    // 24: ebreak
    p.w(addi(1, 0, 1))
        .w(b(8, 0, 1, 0))
        .w(b(8, 0, 1, 1))
        .w(addi(2, 0, 99))
        .w(jal(3, 8))
        .w(addi(4, 0, 99))
        .w(EBREAK);
    let mut m = p.mcu();
    run(&mut m, 1000);
    assert_eq!(m.reg(2), 0);
    assert_eq!(m.reg(4), 0);
    assert_eq!(m.reg(3), FLASH_BASE + 20);
    assert_eq!(m.pc(), FLASH_BASE + 24);
}

#[test]
fn jalr_clears_bit_zero_and_reads_rs1_before_link() {
    let mut p = Prog::default();
    // jalr x1, 1(x1): цель считается из старого x1, младший бит цели обнуляется.
    p.ws(&li(1, FLASH_BASE + 16)) // 0, 4
        .w(jalr(1, 1, 1)) // 8 → 17 & !1 = 16
        .w(EBREAK) // 12
        .w(addi(2, 0, 5)) // 16
        .w(EBREAK);
    let mut m = p.mcu();
    run(&mut m, 1000);
    assert_eq!(m.reg(2), 5);
    assert_eq!(m.reg(1), FLASH_BASE + 12);
}

#[test]
fn compressed_instructions_execute() {
    let mut p = Prog::default();
    p.ws(&li(2, RAM_BASE + 0x100)) // sp
        .h(0x1141) // c.addi sp, -16
        .h(0x4505) // c.li a0, 1
        .h(0x050e) // c.slli a0, 3 → 8
        .h(0x85aa) // c.mv a1, a0
        .h(0x952e) // c.add a0, a1 → 16
        .h(0xc62a) // c.swsp a0, 12(sp)
        .h(0x4632) // c.lwsp a2, 12(sp)
        .h(0x8a3d) // c.andi a2, 15 → 0
        .h(0x0141) // c.addi sp, 16
        .w(EBREAK);
    let mut m = p.mcu();
    run(&mut m, 1000);
    assert_eq!(m.reg(10), 16, "a0");
    assert_eq!(m.reg(11), 8, "a1");
    assert_eq!(m.reg(12), 0, "a2");
    assert_eq!(m.reg(2), RAM_BASE + 0x100, "sp");
}

#[test]
fn decoder_matches_llvm_assembler() {
    let pc = 0x4200_2000;
    for v in sim_mcu::llvm_vectors::VECTORS {
        let d = decode(v.raw, pc);
        let imm = if v.pc_rel {
            pc.wrapping_add(v.imm as i32 as u32)
        } else {
            v.imm as i32 as u32
        };
        assert_eq!(
            (d.op, d.rd, d.rs1, d.rs2, d.len, d.imm),
            (v.op, v.rd, v.rs1, v.rs2, v.len, imm),
            "{} (0x{:x})",
            v.asm,
            v.raw
        );
    }
}

#[test]
fn reserved_and_unsupported_encodings_are_illegal() {
    let cases: &[(u32, &str)] = &[
        (0x0000, "полуслово из нулей"),
        (0x0004, "c.addi4spn с nzuimm = 0"),
        (0x2000, "c.fld (нет D)"),
        (0x6000, "c.flw (нет F)"),
        (0x8000, "зарезервированный код квадранта 0 (Zcb нет)"),
        (0x6101, "c.addi16sp с nzimm = 0"),
        (0x6501, "c.lui с nzimm = 0"),
        (0x9105, "c.srli с shamt[5] = 1 в RV32"),
        (0x9d0d, "c.subw (только RV64)"),
        (0x2002, "c.fldsp"),
        (0x4002, "c.lwsp с rd = 0"),
        (0x8002, "c.jr с rs1 = 0"),
        (0xffff_ffff, "все единицы"),
        (0x0000_007f, "неизвестный opcode"),
        (0x0200_1013, "slli с shamt[5] = 1 в RV32"),
        (0x0020_0073, "uret (нет U-режима)"),
        (0x0000_3003, "ld (RV64)"),
        (0x0000_4023, "запись с funct3 = 100"),
        (0x4000_1033, "sll с funct7 = 0100000"),
        (0x0000_4073, "SYSTEM funct3 = 100"),
    ];
    for &(raw, what) in cases {
        assert_eq!(decode(raw, FLASH_BASE).op, Op::Illegal, "{what}: 0x{raw:x}");
    }
}

#[test]
fn illegal_instruction_trap_sets_mepc_mcause_mtval() {
    let mut p = Prog::default();
    set_mtvec(&mut p, 0x80, 0); // 0, 4, 8
    p.w(0x0000_007f) // 12: недопустимая
        .at(0x80)
        .w(csrrs(10, MCAUSE, 0))
        .w(csrrs(11, MEPC, 0))
        .w(csrrs(12, MTVAL, 0))
        .w(EBREAK);
    let mut m = p.mcu();
    run(&mut m, 1000);
    assert_eq!(m.reg(10), cause::ILLEGAL_INSTRUCTION);
    assert_eq!(m.reg(11), FLASH_BASE + 12);
    assert_eq!(m.reg(12), 0x7f);
    assert_eq!(m.exception_count(cause::ILLEGAL_INSTRUCTION), 1);
}

#[test]
fn misaligned_and_bad_accesses_trap_with_address() {
    let cases: [(u32, Vec<u32>, u32); 4] = [
        // (адрес, инструкция доступа через x6, ожидаемый mcause)
        (RAM_BASE + 2, vec![lw(7, 6, 0)], cause::LOAD_MISALIGNED),
        (RAM_BASE + 1, vec![s(0, 0, 6, 1)], cause::STORE_MISALIGNED),
        (0x1000_0000, vec![lw(7, 6, 0)], cause::LOAD_ACCESS_FAULT),
        (FLASH_BASE, vec![sw(0, 6, 0)], cause::STORE_ACCESS_FAULT),
    ];
    for (addr, ins, want) in cases {
        let mut p = Prog::default();
        set_mtvec(&mut p, 0x80, 0);
        p.ws(&li(6, addr)).ws(&ins).w(EBREAK).at(0x80);
        p.w(csrrs(10, MCAUSE, 0)).w(csrrs(12, MTVAL, 0)).w(EBREAK);
        let mut m = p.mcu();
        run(&mut m, 1000);
        assert_eq!(m.reg(10), want, "mcause для 0x{addr:x}");
        assert_eq!(m.reg(12), addr, "mtval для 0x{addr:x}");
    }
}

#[test]
fn fetch_from_unmapped_memory_is_instruction_access_fault() {
    let mut p = Prog::default();
    set_mtvec(&mut p, 0x80, 0);
    p.ws(&li(6, 0x1000_0000)).w(jalr(0, 6, 0)).at(0x80);
    p.w(csrrs(10, MCAUSE, 0))
        .w(csrrs(11, MEPC, 0))
        .w(csrrs(12, MTVAL, 0))
        .w(EBREAK);
    let mut m = p.mcu();
    run(&mut m, 1000);
    assert_eq!(m.reg(10), cause::INSTR_ACCESS_FAULT);
    assert_eq!(m.reg(11), 0x1000_0000);
    assert_eq!(m.reg(12), 0x1000_0000);
}

#[test]
fn trap_without_handler_is_reported_not_panicking() {
    // mtvec = 0: исключение уводит выборку в пустоту → двойной сбой.
    let mut p = Prog::default();
    p.w(addi(1, 0, 1)).w(0xffff_ffff);
    let mut m = p.mcu();
    let r = m.run_for(1e-3).expect("шаг");
    assert!(r.halted);
    let f = m.fault().expect("сбой должен быть сообщён");
    assert_eq!(f.kind, FaultKind::InstructionAccessFault);
    assert_eq!(f.pc, 0);
    assert_eq!(
        f.source,
        FaultSource::TrapLoop {
            first_cause: cause::ILLEGAL_INSTRUCTION,
            first_pc: FLASH_BASE + 4
        }
    );
    assert!(!f.to_string().is_empty());
    // Время модели продолжает идти, ядро стоит.
    m.run_for(1e-3).expect("шаг");
    assert!((m.time() - 2e-3).abs() < 1e-9);
}

#[test]
fn repeated_identical_trap_is_detected_as_loop() {
    // Обработчик возвращается на ту же недопустимую инструкцию (не сдвигая mepc).
    let mut p = Prog::default();
    set_mtvec(&mut p, 0x80, 0);
    p.w(0xffff_ffff).at(0x80).w(MRET);
    let mut m = p.mcu();
    m.run_for(1e-3).expect("шаг");
    let f = m.fault().expect("зацикливание должно быть обнаружено");
    assert_eq!(f.kind, FaultKind::IllegalInstruction);
    assert_eq!(f.pc, FLASH_BASE + 12);
    assert!(matches!(f.source, FaultSource::TrapLoop { .. }));
}

#[test]
fn firmware_reported_fault_reaches_host() {
    let mut p = Prog::default();
    p.ws(&li(6, 0x6000_0000))
        .ws(&li(7, 0x1234))
        .w(sw(7, 6, 0x20)) // FAULT_PC
        .w(addi(7, 0, 0x56))
        .w(sw(7, 6, 0x24)) // FAULT_TVAL
        .w(addi(7, 0, 5))
        .w(sw(7, 6, 0x28)) // FAULT_CAUSE → остановка
        .w(addi(8, 0, 1));
    let mut m = p.mcu();
    m.run_for(1e-4).expect("шаг");
    let f = m.fault().expect("сбой");
    assert_eq!((f.cause, f.pc, f.tval), (5, 0x1234, 0x56));
    assert_eq!(f.source, FaultSource::Firmware);
    assert_eq!(m.reg(8), 0, "после сообщения ядро стоит");
}

#[test]
fn ebreak_halts_to_host_and_exit_register_sets_code() {
    let mut p = Prog::default();
    p.w(addi(1, 0, 3)).w(EBREAK);
    let mut m = p.mcu();
    assert_eq!(
        run(&mut m, 100),
        HaltReason::Breakpoint { pc: FLASH_BASE + 4 }
    );

    let mut p = Prog::default();
    p.ws(&li(6, 0x6000_0000))
        .w(addi(7, 0, 42))
        .w(sw(7, 6, 0x18));
    let mut m = p.mcu();
    m.run_for(1e-5).expect("шаг");
    assert_eq!(m.exit_code(), Some(42));
}

#[test]
fn csr_semantics() {
    let mut p = Prog::default();
    set_mtvec(&mut p, 0x80, 0);
    p.w(csrrs(10, 0xf14, 0)) // csrrs a0, mhartid, x0 — чтение RO без записи допустимо
        .w(csrrs(11, 0x301, 0)) // misa
        .w(addi(5, 0, 8))
        .w(csrrw(0, MSTATUS, 5)) // MIE = 1
        .w(csrrs(12, MSTATUS, 0)) // MPP = 11 читается всегда
        .w(csrrw(0, 0xf14, 5)) // запись в mhartid → недопустимая
        .w(EBREAK)
        .at(0x80)
        .w(csrrs(13, MCAUSE, 0))
        .w(csrrs(14, MSTATUS, 0)) // после входа в ловушку: MIE = 0, MPIE = 1
        .w(EBREAK);
    let mut m = p.mcu();
    run(&mut m, 1000);
    assert_eq!(m.reg(10), 0);
    assert_eq!(m.reg(11), 0x4000_1104);
    assert_eq!(m.reg(12), 0x1808);
    assert_eq!(m.reg(13), cause::ILLEGAL_INSTRUCTION);
    assert_eq!(m.reg(14), 0x1880);
}

#[test]
fn mret_restores_interrupt_enable() {
    let mut p = Prog::default();
    // mstatus = MPIE, mepc = метка 0x40 → mret: MIE ← MPIE = 1, MPIE ← 1.
    p.w(addi(5, 0, 0x80))
        .w(csrrw(0, MSTATUS, 5))
        .ws(&li(6, FLASH_BASE + 0x40))
        .w(csrrw(0, MEPC, 6))
        .w(MRET)
        .at(0x40)
        .w(csrrs(10, MSTATUS, 0))
        .w(EBREAK);
    let mut m = p.mcu();
    run(&mut m, 1000);
    assert_eq!(m.reg(10) & 0x88, 0x88);
}

#[test]
fn timer_interrupt_in_vectored_mode_jumps_to_base_plus_4x7() {
    let mut p = Prog::default();
    set_mtvec(&mut p, 0x100, 1); // векторный режим
    // mtimecmp = 100 (mtime стартует с 0, 100 тиков = 1000 тактов).
    p.ws(&li(6, 0x0200_4000))
        .w(addi(7, 0, 100))
        .w(sw(0, 6, 4))
        .w(sw(7, 6, 0))
        .w(addi(5, 0, 0x80))
        .w(csrrw(0, MIE, 5)) // MTIE
        .w(addi(5, 0, 8))
        .w(csrrw(0, MSTATUS, 5)) // MIE
        .w(WFI)
        .w(EBREAK)
        .at(0x100 + 4 * 7)
        .w(csrrs(12, 0xb00, 0)) // mcycle на входе в обработчик
        .w(csrrs(10, MCAUSE, 0))
        .w(csrrs(11, MEPC, 0))
        .w(EBREAK);
    let mut m = p.mcu();
    run(&mut m, 100_000);
    assert_eq!(m.reg(10), cause::INTERRUPT | cause::M_TIMER_INT);
    assert_eq!(
        m.reg(11),
        FLASH_BASE + 4 * 13,
        "mepc — инструкция после wfi"
    );
    let c = m.reg(12);
    assert!(
        (1000..1020).contains(&c),
        "сон промотан до события (mtime = 100): {c}"
    );
}

#[test]
fn wfi_wakes_on_pending_interrupt_even_with_mie_clear() {
    let mut p = Prog::default();
    p.ws(&li(6, 0x0200_4000))
        .ws(&li(7, 16_000)) // 1 мс
        .w(sw(0, 6, 4))
        .w(sw(7, 6, 0))
        .w(addi(5, 0, 0x80))
        .w(csrrw(0, MIE, 5)) // MTIE разрешён, но mstatus.MIE = 0
        .w(WFI)
        .w(csrrs(11, 0xb00, 0)) // mcycle после пробуждения
        .w(addi(10, 0, 1))
        .w(EBREAK);
    let mut m = p.mcu();
    let wall = std::time::Instant::now();
    run(&mut m, 170_000);
    assert_eq!(
        m.reg(10),
        1,
        "после пробуждения исполнение продолжается без ловушки"
    );
    assert_eq!(m.interrupt_count(cause::M_TIMER_INT), 0);
    let c = m.reg(11);
    assert!(
        (160_000..160_010).contains(&c),
        "пробуждение на 1 мс = 160 000 тактов: {c}"
    );
    assert!(
        m.active_cycles() < 200,
        "сон не считается загрузкой: {}",
        m.active_cycles()
    );
    assert!(
        wall.elapsed().as_millis() < 50,
        "сон промотан, а не исполнен"
    );
}

#[test]
fn ecall_traps_with_cause_11_and_mepc_at_ecall() {
    let mut p = Prog::default();
    set_mtvec(&mut p, 0x80, 0);
    p.w(ECALL).w(EBREAK).at(0x80);
    p.w(csrrs(10, MCAUSE, 0)).w(csrrs(11, MEPC, 0)).w(EBREAK);
    let mut m = p.mcu();
    run(&mut m, 1000);
    assert_eq!(m.reg(10), cause::ECALL_M);
    assert_eq!(m.reg(11), FLASH_BASE + 12);
}

#[test]
fn code_runs_from_ram_through_slow_fetch_path() {
    // Записать в ОЗУ «addi x5, x0, 42; c.li a0, 7; ebreak» и перейти туда.
    let mut p = Prog::default();
    p.ws(&li(6, RAM_BASE + 0x40))
        .ws(&li(7, addi(5, 0, 42)))
        .w(sw(7, 6, 0))
        .ws(&li(7, 0x0073_451d)) // c.li a0, 7 (0x451d) + младшая половина ebreak (0x0073)
        .w(sw(7, 6, 4))
        .ws(&li(7, 0x0010))
        .w(s(8, 7, 6, 1)) // sh: старшая половина ebreak
        .w(jalr(0, 6, 0));
    let mut m = p.mcu();
    let h = run(&mut m, 1000);
    assert_eq!(m.reg(5), 42);
    assert_eq!(m.reg(10), 7);
    assert_eq!(
        h,
        HaltReason::Breakpoint {
            pc: RAM_BASE + 0x46
        }
    );
}

#[test]
fn mcycle_and_minstret_count_and_are_writable() {
    let mut p = Prog::default();
    p.w(csrrs(10, 0xb02, 0)) // minstret
        .w(NOP)
        .w(NOP)
        .w(csrrs(11, 0xb02, 0))
        .ws(&li(5, 1_000_000))
        .w(csrrw(0, 0xb00, 5)) // mcycle ← 10⁶
        .w(csrrs(12, 0xb00, 0))
        .w(EBREAK);
    let mut m = p.mcu();
    run(&mut m, 1000);
    assert_eq!(m.reg(11) - m.reg(10), 3);
    assert!(m.reg(12) > 1_000_000 && m.reg(12) < 1_000_010);
}

#[test]
fn watchdog_expiry_resets_chip_and_counts() {
    // Включить сторож на 100 мкс и зависнуть.
    let mut p = Prog::default();
    p.ws(&li(6, 0x6000_6000))
        .w(addi(7, 0, 100))
        .w(sw(7, 6, 4))
        .w(addi(7, 0, 1))
        .w(sw(7, 6, 0))
        .w(jal(0, 0)); // бесконечный цикл
    let mut m = p.mcu();
    m.run_for(250e-6).expect("шаг");
    assert_eq!(m.watchdog_resets(), 2, "сброс каждые 100 мкс");
    assert_eq!(m.reset_reason(), sim_mcu::ResetReason::Watchdog);
    m.reset();
    assert_eq!(m.watchdog_resets(), 0, "сброс по питанию обнуляет счётчик");
}

#[test]
fn host_api_rejects_bad_arguments() {
    let mut m = Prog::default().w(EBREAK).mcu();
    assert!(matches!(m.run_for(-1.0), Err(McuError::BadDuration(_))));
    assert!(matches!(m.run_for(f64::NAN), Err(McuError::BadDuration(_))));
    assert!(matches!(
        m.write_config(&[0; 5000]),
        Err(McuError::ConfigTooLarge(5000))
    ));
    assert!(matches!(
        Mcu::from_elf(b"not an elf"),
        Err(McuError::Elf(_))
    ));
    let mut elf = sim_mcu::firmware::selftest_elf().to_vec();
    elf[18] = 0x3e; // e_machine = x86-64
    assert!(matches!(Mcu::from_elf(&elf), Err(McuError::Elf(_))));
    assert!(Mcu::from_elf(&sim_mcu::firmware::selftest_elf()[..40]).is_err());
    assert!(matches!(
        Mcu::from_image(&[0x13, 0, 0, 0], 0x1000),
        Err(McuError::EntryOutOfRange(_))
    ));
    // Неверные каналы и выводы — игнорируются, без паники.
    m.set_adc_volts(99, 1.0);
    m.set_gpio_input(77, false);
    assert!(!m.gpio_output(77));
    assert_eq!(m.pwm_duty(9), 0.0);
}
