//! Ядро процессора: RV32I + M + C + Zicsr, только машинный режим (M-mode).
//!
//! Написано по спецификации RISC-V: «The RISC-V Instruction Set Manual, Volume I:
//! Unprivileged ISA» (20191213: RV32I, M, C — таблицы 16.5–16.7 для сжатых
//! инструкций, Zicsr) и «Volume II: Privileged Architecture» (20211203: mstatus,
//! mtvec, mepc, mcause, mtval, mie/mip, порядок приоритетов прерываний). Код других
//! эмуляторов не использовался.
//!
//! Устройство: каждая инструкция декодируется один раз в компактную запись
//! [`Decoded`] (сжатые разворачиваются в базовые операции, адреса переходов и
//! AUIPC считаются сразу абсолютными); для флеша записи кэшируются по адресу —
//! флеш с точки зрения ядра только читается, поэтому кэш не устаревает.
//!
//! **Модель времени (приближённая, «оценка»)**: 1 такт на инструкцию плюс добавки
//! [`timing`]. Это не потактовая модель конвейера ESP32-C3: кэш флеша считается
//! всегда попадающим, ожидание шины не моделируется. Для оценки загрузки ЦП и
//! задержек прерываний точности ±30 % достаточно.

use crate::bus::{Bus, FLASH_BASE, RAM_BASE, RAM_SIZE};

/// Коды причин ловушек (`mcause`), Privileged ISA, таблица 3.6.
pub mod cause {
    pub const INSTR_ACCESS_FAULT: u32 = 1;
    pub const ILLEGAL_INSTRUCTION: u32 = 2;
    pub const BREAKPOINT: u32 = 3;
    pub const LOAD_MISALIGNED: u32 = 4;
    pub const LOAD_ACCESS_FAULT: u32 = 5;
    pub const STORE_MISALIGNED: u32 = 6;
    pub const STORE_ACCESS_FAULT: u32 = 7;
    pub const ECALL_M: u32 = 11;
    /// Старший бит `mcause` — прерывание (а не исключение).
    pub const INTERRUPT: u32 = 1 << 31;
    pub const M_SOFTWARE_INT: u32 = 3;
    pub const M_TIMER_INT: u32 = 7;
    pub const M_EXTERNAL_INT: u32 = 11;
}

/// Добавочные такты сверх базового 1 такта на инструкцию («оценка», по порядку
/// величины для 4-стадийного конвейера ESP32-C3).
pub mod timing {
    /// Загрузка из памяти (lb/lh/lw/lbu/lhu): +1 такт.
    pub const LOAD_EXTRA: u64 = 1;
    /// Доступ к регистрам периферии (загрузка или запись): ещё +4 такта.
    pub const MMIO_EXTRA: u64 = 4;
    /// Выполненный условный переход: +2 такта (сброс конвейера).
    pub const BRANCH_TAKEN_EXTRA: u64 = 2;
    /// Безусловный переход jal/jalr: +2 такта.
    pub const JUMP_EXTRA: u64 = 2;
    /// Умножение mul/mulh/mulhsu/mulhu: +1 такт.
    pub const MUL_EXTRA: u64 = 1;
    /// Деление div/divu/rem/remu: +32 такта (поразрядный делитель).
    pub const DIV_EXTRA: u64 = 32;
    /// Вход в ловушку/прерывание: +4 такта; mret: +2.
    pub const TRAP_ENTRY: u64 = 4;
    pub const MRET_EXTRA: u64 = 2;
    /// Чтение/запись CSR: +1 такт.
    pub const CSR_EXTRA: u64 = 1;
}

/// Значение `misa`: MXL=1 (32 бита), расширения C (бит 2), I (бит 8), M (бит 12).
pub const MISA: u32 = (1 << 30) | (1 << 2) | (1 << 8) | (1 << 12);

/// Биты `mstatus`.
pub const MSTATUS_MIE: u32 = 1 << 3;
pub const MSTATUS_MPIE: u32 = 1 << 7;
/// MPP всегда 11 (машинный режим — единственный).
pub const MSTATUS_MPP: u32 = 3 << 11;

/// Биты `mie`/`mip`.
pub const MIP_MSIP: u32 = 1 << 3;
pub const MIP_MTIP: u32 = 1 << 7;
pub const MIP_MEIP: u32 = 1 << 11;

/// Операция после декодирования. Сжатые инструкции разворачиваются в эти же
/// операции; LUI/AUIPC/C.LI/C.LUI — в `Li` с готовой константой.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Op {
    /// Недопустимая инструкция; `imm` — её биты (для `mtval`).
    Illegal,
    /// rd ← imm (LUI, AUIPC с уже посчитанным адресом, C.LI, C.LUI).
    Li,
    /// rd ← pc+len; pc ← imm (абсолютный адрес).
    Jal,
    /// rd ← pc+len; pc ← (rs1+imm) & !1.
    Jalr,
    Beq,
    Bne,
    Blt,
    Bge,
    Bltu,
    Bgeu,
    Lb,
    Lh,
    Lw,
    Lbu,
    Lhu,
    Sb,
    Sh,
    Sw,
    Addi,
    Slti,
    Sltiu,
    Xori,
    Ori,
    Andi,
    Slli,
    Srli,
    Srai,
    Add,
    Sub,
    Sll,
    Slt,
    Sltu,
    Xor,
    Srl,
    Sra,
    Or,
    And,
    Mul,
    Mulh,
    Mulhsu,
    Mulhu,
    Div,
    Divu,
    Rem,
    Remu,
    Fence,
    FenceI,
    Ecall,
    Ebreak,
    Mret,
    Wfi,
    Csrrw,
    Csrrs,
    Csrrc,
    Csrrwi,
    Csrrsi,
    Csrrci,
}

/// Декодированная инструкция. `rd/rs1/rs2` — архитектурные номера регистров
/// (0..=31), `len` — длина в байтах (2 или 4), `imm` — непосредственное значение
/// (для переходов — абсолютный адрес цели, для CSR — номер CSR, для `Illegal` —
/// биты инструкции).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Decoded {
    pub op: Op,
    pub rd: u8,
    pub rs1: u8,
    pub rs2: u8,
    pub len: u8,
    pub imm: u32,
}

impl Decoded {
    fn illegal(raw: u32, len: u8) -> Decoded {
        Decoded {
            op: Op::Illegal,
            rd: 0,
            rs1: 0,
            rs2: 0,
            len,
            imm: raw,
        }
    }
}

/// Номер «регистра-стока»: запись в x0 перенаправляется сюда, поэтому x0 всегда 0
/// без проверки на каждой инструкции.
const SINK: u8 = 32;

/// Знаковое расширение младших `bits` бит.
#[inline]
fn sext(x: u32, bits: u32) -> u32 {
    let s = 32 - bits;
    (((x << s) as i32) >> s) as u32
}

/// Декодирует инструкцию. `raw` — 32 бита начиная с `pc` (для сжатой берутся
/// младшие 16). Признак длины — младшие два бита: `11` — 32-битная, иначе сжатая.
pub fn decode(raw: u32, pc: u32) -> Decoded {
    if raw & 3 == 3 {
        decode32(raw, pc)
    } else {
        decode16(raw & 0xffff, pc)
    }
}

fn decode32(i: u32, pc: u32) -> Decoded {
    use Op::*;
    let opcode = i & 0x7f;
    let rd = ((i >> 7) & 31) as u8;
    let f3 = (i >> 12) & 7;
    let rs1 = ((i >> 15) & 31) as u8;
    let rs2 = ((i >> 20) & 31) as u8;
    let f7 = i >> 25;
    let imm_i = ((i as i32) >> 20) as u32;
    let imm_s = ((((i as i32) >> 25) << 5) as u32) | ((i >> 7) & 31);
    let imm_b = ((((i as i32) >> 31) << 12) as u32)
        | (((i >> 7) & 1) << 11)
        | (((i >> 25) & 0x3f) << 5)
        | (((i >> 8) & 0xf) << 1);
    let imm_u = i & 0xffff_f000;
    let imm_j = ((((i as i32) >> 31) << 20) as u32)
        | (i & 0x000f_f000)
        | (((i >> 20) & 1) << 11)
        | (((i >> 21) & 0x3ff) << 1);
    let ill = Decoded::illegal(i, 4);
    let mk = |op, rd, rs1, rs2, imm| Decoded {
        op,
        rd,
        rs1,
        rs2,
        len: 4,
        imm,
    };
    match opcode {
        0x37 => mk(Li, rd, 0, 0, imm_u),
        0x17 => mk(Li, rd, 0, 0, pc.wrapping_add(imm_u)),
        0x6f => mk(Jal, rd, 0, 0, pc.wrapping_add(imm_j)),
        0x67 if f3 == 0 => mk(Jalr, rd, rs1, 0, imm_i),
        0x63 => {
            let op = match f3 {
                0 => Beq,
                1 => Bne,
                4 => Blt,
                5 => Bge,
                6 => Bltu,
                7 => Bgeu,
                _ => return ill,
            };
            mk(op, 0, rs1, rs2, pc.wrapping_add(imm_b))
        }
        0x03 => {
            let op = match f3 {
                0 => Lb,
                1 => Lh,
                2 => Lw,
                4 => Lbu,
                5 => Lhu,
                _ => return ill,
            };
            mk(op, rd, rs1, 0, imm_i)
        }
        0x23 => {
            let op = match f3 {
                0 => Sb,
                1 => Sh,
                2 => Sw,
                _ => return ill,
            };
            mk(op, 0, rs1, rs2, imm_s)
        }
        0x13 => {
            let op = match f3 {
                0 => Addi,
                2 => Slti,
                3 => Sltiu,
                4 => Xori,
                6 => Ori,
                7 => Andi,
                // В RV32 shamt[5] (бит 25) обязан быть 0 — он входит в проверку f7.
                1 if f7 == 0 => return mk(Slli, rd, rs1, 0, rs2 as u32),
                5 if f7 == 0 => return mk(Srli, rd, rs1, 0, rs2 as u32),
                5 if f7 == 0x20 => return mk(Srai, rd, rs1, 0, rs2 as u32),
                _ => return ill,
            };
            mk(op, rd, rs1, 0, imm_i)
        }
        0x33 => {
            let op = match (f7, f3) {
                (0, 0) => Add,
                (0x20, 0) => Sub,
                (0, 1) => Sll,
                (0, 2) => Slt,
                (0, 3) => Sltu,
                (0, 4) => Xor,
                (0, 5) => Srl,
                (0x20, 5) => Sra,
                (0, 6) => Or,
                (0, 7) => And,
                (1, 0) => Mul,
                (1, 1) => Mulh,
                (1, 2) => Mulhsu,
                (1, 3) => Mulhu,
                (1, 4) => Div,
                (1, 5) => Divu,
                (1, 6) => Rem,
                (1, 7) => Remu,
                _ => return ill,
            };
            mk(op, rd, rs1, rs2, 0)
        }
        0x0f => match f3 {
            // FENCE: у нас один такт и одно ядро — порядок памяти и так строгий.
            0 => mk(Fence, 0, 0, 0, 0),
            // FENCE.I (Zifencei): кэш декодирования ведётся только для флеша, который
            // ядро не пишет, — сбрасывать нечего.
            1 => mk(FenceI, 0, 0, 0, 0),
            _ => ill,
        },
        0x73 => match f3 {
            0 => match i {
                0x0000_0073 => mk(Ecall, 0, 0, 0, 0),
                0x0010_0073 => mk(Ebreak, 0, 0, 0, 0),
                0x3020_0073 => mk(Mret, 0, 0, 0, 0),
                0x1050_0073 => mk(Wfi, 0, 0, 0, 0),
                _ => ill,
            },
            1 => mk(Csrrw, rd, rs1, 0, i >> 20),
            2 => mk(Csrrs, rd, rs1, 0, i >> 20),
            3 => mk(Csrrc, rd, rs1, 0, i >> 20),
            5 => mk(Csrrwi, rd, rs1, 0, i >> 20),
            6 => mk(Csrrsi, rd, rs1, 0, i >> 20),
            7 => mk(Csrrci, rd, rs1, 0, i >> 20),
            _ => ill,
        },
        _ => ill,
    }
}

/// Смещение C.J/C.JAL: offset[11|4|9:8|10|6|7|3:1|5] = inst[12|11|10:9|8|7|6|5:3|2].
fn cj_offset(i: u32) -> u32 {
    let v = (((i >> 12) & 1) << 11)
        | (((i >> 11) & 1) << 4)
        | (((i >> 9) & 3) << 8)
        | (((i >> 8) & 1) << 10)
        | (((i >> 7) & 1) << 6)
        | (((i >> 6) & 1) << 7)
        | (((i >> 3) & 7) << 1)
        | (((i >> 2) & 1) << 5);
    sext(v, 12)
}

/// Смещение C.BEQZ/C.BNEZ: offset[8|4:3] = inst[12|11:10], offset[7:6|2:1|5] = inst[6:5|4:3|2].
fn cb_offset(i: u32) -> u32 {
    let v = (((i >> 12) & 1) << 8)
        | (((i >> 10) & 3) << 3)
        | (((i >> 5) & 3) << 6)
        | (((i >> 3) & 3) << 1)
        | (((i >> 2) & 1) << 5);
    sext(v, 9)
}

/// Сжатые инструкции RV32C (Volume I, гл. 16, таблицы 16.5–16.7). Коды для F/D
/// (C.FLD, C.FLW, …), RV64-only и зарезервированные — недопустимые.
fn decode16(i: u32, pc: u32) -> Decoded {
    use Op::*;
    let ill = Decoded::illegal(i, 2);
    let quadrant = i & 3;
    let f3 = (i >> 13) & 7;
    let rdp = (8 + ((i >> 2) & 7)) as u8; // rd'/rs2' (биты 4:2)
    let rs1p = (8 + ((i >> 7) & 7)) as u8; // rs1'/rd' (биты 9:7)
    let rd = ((i >> 7) & 31) as u8; // rd/rs1 (биты 11:7)
    let rs2 = ((i >> 2) & 31) as u8; // rs2 (биты 6:2)
    let bit12 = (i >> 12) & 1;
    let imm6 = sext((bit12 << 5) | ((i >> 2) & 0x1f), 6);
    let shamt = (i >> 2) & 0x1f;
    let c = |op, rd, rs1, rs2, imm| Decoded {
        op,
        rd,
        rs1,
        rs2,
        len: 2,
        imm,
    };
    match (quadrant, f3) {
        // C.ADDI4SPN: addi rd', x2, nzuimm; nzuimm[5:4|9:6|2|3] = inst[12:11|10:7|6|5].
        (0, 0) => {
            let nzuimm = (((i >> 11) & 3) << 4)
                | (((i >> 7) & 0xf) << 6)
                | (((i >> 6) & 1) << 2)
                | (((i >> 5) & 1) << 3);
            if nzuimm == 0 {
                ill // в том числе полностью нулевое полуслово
            } else {
                c(Addi, rdp, 2, 0, nzuimm)
            }
        }
        // C.LW / C.SW: offset[5:3|2|6] = inst[12:10|6|5].
        (0, 2) | (0, 6) => {
            let off = (((i >> 10) & 7) << 3) | (((i >> 6) & 1) << 2) | (((i >> 5) & 1) << 6);
            if f3 == 2 {
                c(Lw, rdp, rs1p, 0, off)
            } else {
                c(Sw, 0, rs1p, rdp, off)
            }
        }
        // C.NOP / C.ADDI (nzimm = 0 — HINT, исполняется как addi).
        (1, 0) => c(Addi, rd, rd, 0, imm6),
        // C.JAL (только RV32): jal x1, offset.
        (1, 1) => c(Jal, 1, 0, 0, pc.wrapping_add(cj_offset(i))),
        // C.LI: addi rd, x0, imm (rd = 0 — HINT).
        (1, 2) => c(Li, rd, 0, 0, imm6),
        (1, 3) => {
            if rd == 2 {
                // C.ADDI16SP: nzimm[9|4|6|8:7|5] = inst[12|6|5|4:3|2].
                let v = (bit12 << 9)
                    | (((i >> 6) & 1) << 4)
                    | (((i >> 5) & 1) << 6)
                    | (((i >> 3) & 3) << 7)
                    | (((i >> 2) & 1) << 5);
                if v == 0 {
                    ill
                } else {
                    c(Addi, 2, 2, 0, sext(v, 10))
                }
            } else {
                // C.LUI: nzimm[17|16:12] = inst[12|6:2]; nzimm = 0 — зарезервировано.
                let v = (bit12 << 17) | (((i >> 2) & 0x1f) << 12);
                if v == 0 {
                    ill
                } else {
                    c(Li, rd, 0, 0, sext(v, 18))
                }
            }
        }
        (1, 4) => match (i >> 10) & 3 {
            // C.SRLI / C.SRAI: в RV32 shamt[5] = inst[12] обязан быть 0.
            0 if bit12 == 0 => c(Srli, rs1p, rs1p, 0, shamt),
            1 if bit12 == 0 => c(Srai, rs1p, rs1p, 0, shamt),
            2 => c(Andi, rs1p, rs1p, 0, imm6),
            3 if bit12 == 0 => {
                let op = match (i >> 5) & 3 {
                    0 => Sub,
                    1 => Xor,
                    2 => Or,
                    _ => And,
                };
                c(op, rs1p, rs1p, rdp, 0)
            }
            // C.SUBW/C.ADDW (RV64) и shamt[5]=1 — недопустимы в RV32C.
            _ => ill,
        },
        // C.J: jal x0, offset.
        (1, 5) => c(Jal, 0, 0, 0, pc.wrapping_add(cj_offset(i))),
        // C.BEQZ / C.BNEZ.
        (1, 6) => c(Beq, 0, rs1p, 0, pc.wrapping_add(cb_offset(i))),
        (1, 7) => c(Bne, 0, rs1p, 0, pc.wrapping_add(cb_offset(i))),
        // C.SLLI (rd = 0 — HINT; shamt[5] = 1 — недопустимо в RV32).
        (2, 0) => {
            if bit12 != 0 {
                ill
            } else {
                c(Slli, rd, rd, 0, shamt)
            }
        }
        // C.LWSP: offset[5|4:2|7:6] = inst[12|6:4|3:2]; rd = 0 — зарезервировано.
        (2, 2) => {
            if rd == 0 {
                ill
            } else {
                let off = (bit12 << 5) | (((i >> 4) & 7) << 2) | (((i >> 2) & 3) << 6);
                c(Lw, rd, 2, 0, off)
            }
        }
        (2, 4) => match (bit12, rd, rs2) {
            (0, 0, 0) => ill,                   // C.JR с rs1 = 0 — зарезервировано
            (0, _, 0) => c(Jalr, 0, rd, 0, 0),  // C.JR
            (0, _, _) => c(Add, rd, 0, rs2, 0), // C.MV
            (_, 0, 0) => c(Ebreak, 0, 0, 0, 0), // C.EBREAK
            (_, _, 0) => c(Jalr, 1, rd, 0, 0),  // C.JALR
            _ => c(Add, rd, rd, rs2, 0),        // C.ADD
        },
        // C.SWSP: offset[5:2|7:6] = inst[12:9|8:7].
        (2, 6) => {
            let off = (((i >> 9) & 0xf) << 2) | (((i >> 7) & 3) << 6);
            c(Sw, 0, 2, rs2, off)
        }
        _ => ill,
    }
}

/// Машинные CSR.
#[derive(Clone, Debug, Default)]
pub struct Csrs {
    /// Только MIE и MPIE (MPP читается как 11).
    pub mstatus: u32,
    pub mie: u32,
    /// Отражение линий прерываний (обновляется перед каждой пачкой инструкций).
    pub mip: u32,
    pub mtvec: u32,
    pub mscratch: u32,
    pub mepc: u32,
    pub mcause: u32,
    pub mtval: u32,
    /// Смещения счётчиков после записи прошивкой в mcycle/minstret.
    pub mcycle_offset: u64,
    pub minstret_offset: u64,
}

/// Счётчики времени ядра (общие для CPU и шины).
#[derive(Clone, Copy, Debug, Default)]
pub struct Clock {
    /// Такты с момента создания МК (160 МГц), включая сон в `wfi`.
    pub cycle: u64,
    /// Выполненные инструкции.
    pub instret: u64,
}

/// Почему пачка инструкций закончилась.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Exit {
    /// Исчерпан бюджет тактов.
    Budget,
    /// Изменилось что-то, влияющее на прерывания/события (CSR, MMIO, mret).
    Reeval,
    /// `wfi` без ожидающих прерываний — ядро уснуло.
    Sleep,
    /// `ebreak` — остановка в отладчик (хост).
    Breakpoint,
    /// Неустранимый сбой (зацикливание ловушек) — см. [`Cpu::fault`].
    Fault,
}

/// Сбой, обнаруженный эмулятором (не прошивкой): исключение внутри обработчика
/// исключения или одно и то же исключение по одному адресу много раз подряд.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CpuFault {
    /// Причина последнего исключения (`mcause`).
    pub cause: u32,
    /// Адрес инструкции, вызвавшей последнее исключение.
    pub pc: u32,
    /// `mtval` последнего исключения.
    pub tval: u32,
    /// Причина и адрес первого исключения цепочки.
    pub first_cause: u32,
    pub first_pc: u32,
}

/// Сколько раз подряд одно и то же исключение по одному адресу считается зацикливанием.
const SAME_TRAP_LIMIT: u32 = 64;

/// Состояние ядра.
#[derive(Clone, Debug)]
pub struct Cpu {
    /// x0..x31 и регистр-сток [`SINK`]; размер 64 — индекс маскируется `& 63`
    /// без проверки границ.
    pub x: [u32; 64],
    pub pc: u32,
    pub csr: Csrs,
    /// Ядро спит в `wfi`.
    pub sleeping: bool,
    /// Внутри обработчика исключения (между входом и `mret`).
    in_trap: bool,
    trap_first: (u32, u32),
    last_trap: (u32, u32),
    same_trap_count: u32,
    /// Сбой эмулятора, остановивший ядро.
    pub fault: Option<CpuFault>,
    /// Счётчики взятых прерываний и исключений по коду причины (0..=15) — статистика
    /// для хоста, как счётчики событий в отладчике.
    pub interrupts_taken: [u64; 16],
    pub exceptions_taken: [u64; 16],
    /// Кэш декодирования флеша: по записи на каждое полуслово загруженного образа
    /// (декодируется заранее, при создании; флеш ядро не пишет).
    icache: Vec<Decoded>,
}

/// Это адрес периферии или CLINT (для добавочных тактов доступа).
#[inline]
fn is_mmio(addr: u32) -> bool {
    addr.wrapping_sub(crate::bus::PERIPH_BASE) < crate::bus::PERIPH_SIZE
        || addr.wrapping_sub(crate::bus::CLINT_BASE) < crate::bus::CLINT_SIZE
}

impl Cpu {
    /// Ядро с кэшем декодирования первых `code_len` байт флеша.
    pub fn new(bus: &Bus, code_len: usize) -> Cpu {
        let slots = code_len.div_ceil(2);
        let mut icache = Vec::with_capacity(slots);
        for k in 0..slots {
            let pc = FLASH_BASE + 2 * k as u32;
            let lo = bus.fetch16(pc).unwrap_or(0) as u32;
            let hi = bus.fetch16(pc + 2).unwrap_or(0) as u32;
            icache.push(Self::exec_form(decode(lo | (hi << 16), pc)));
        }
        Cpu {
            x: [0; 64],
            pc: 0,
            csr: Csrs::default(),
            sleeping: false,
            in_trap: false,
            trap_first: (0, 0),
            last_trap: (0, 0),
            same_trap_count: 0,
            fault: None,
            interrupts_taken: [0; 16],
            exceptions_taken: [0; 16],
            icache,
        }
    }

    /// Форма для исполнения: запись в x0 → в регистр-сток.
    #[inline]
    fn exec_form(mut d: Decoded) -> Decoded {
        if d.rd == 0 {
            d.rd = SINK;
        }
        d
    }

    /// Сброс ядра: регистры в 0, pc ← вектор сброса, CSR — по умолчанию.
    /// Кэш декодирования сохраняется (флеш не меняется).
    pub fn reset(&mut self, entry: u32) {
        self.x = [0; 64];
        self.pc = entry;
        self.csr = Csrs::default();
        self.sleeping = false;
        self.in_trap = false;
        self.same_trap_count = 0;
        self.last_trap = (0, 0);
        self.fault = None;
    }

    /// Регистр x0..x31.
    pub fn reg(&self, i: usize) -> u32 {
        if i == 0 { 0 } else { self.x[i & 31] }
    }

    #[inline(always)]
    fn set(&mut self, rd: u8, v: u32) {
        self.x[(rd & 63) as usize] = v;
    }

    #[inline(always)]
    fn get(&self, r: u8) -> u32 {
        self.x[(r & 63) as usize]
    }

    /// Декодирует инструкцию по адресу (медленный путь, без кэша: ОЗУ или флеш за
    /// пределами образа). `Err` — выборка из неисполняемой/несуществующей памяти.
    fn fetch_decode(&self, bus: &Bus, pc: u32) -> Result<Decoded, ()> {
        let lo = bus.fetch16(pc).ok_or(())?;
        let raw = if lo & 3 == 3 {
            let hi = bus.fetch16(pc.wrapping_add(2)).ok_or(())?;
            (lo as u32) | ((hi as u32) << 16)
        } else {
            lo as u32
        };
        Ok(Self::exec_form(decode(raw, pc)))
    }

    /// Вход в ловушку: mepc, mcause, mtval, MPIE ← MIE, MIE ← 0, pc ← mtvec.
    fn enter_trap(&mut self, mcause: u32, tval: u32, epc: u32) {
        let mie = self.csr.mstatus & MSTATUS_MIE;
        self.csr.mstatus = (self.csr.mstatus & !(MSTATUS_MIE | MSTATUS_MPIE))
            | if mie != 0 { MSTATUS_MPIE } else { 0 };
        self.csr.mepc = epc & !1;
        self.csr.mcause = mcause;
        self.csr.mtval = tval;
        let base = self.csr.mtvec & !3;
        let vectored = self.csr.mtvec & 3 == 1;
        self.pc = if mcause & cause::INTERRUPT != 0 && vectored {
            base.wrapping_add(4 * (mcause & 0x7fff_ffff))
        } else {
            base
        };
    }

    /// Синхронное исключение. Возвращает `false`, если ядро остановлено как
    /// зациклившееся (исключение-сбой внутри обработчика или повтор по одному адресу).
    fn exception(&mut self, code: u32, tval: u32, epc: u32, clk: &mut Clock) -> bool {
        let is_fault = !matches!(code, cause::ECALL_M | cause::BREAKPOINT);
        if is_fault {
            if self.in_trap {
                self.fault = Some(CpuFault {
                    cause: code,
                    pc: epc,
                    tval,
                    first_cause: self.trap_first.0,
                    first_pc: self.trap_first.1,
                });
                return false;
            }
            if self.last_trap == (code, epc) {
                self.same_trap_count += 1;
                if self.same_trap_count >= SAME_TRAP_LIMIT {
                    self.fault = Some(CpuFault {
                        cause: code,
                        pc: epc,
                        tval,
                        first_cause: code,
                        first_pc: epc,
                    });
                    return false;
                }
            } else {
                self.last_trap = (code, epc);
                self.same_trap_count = 1;
            }
            self.in_trap = true;
            self.trap_first = (code, epc);
        }
        self.exceptions_taken[(code & 15) as usize] += 1;
        self.enter_trap(code, tval, epc);
        clk.cycle += timing::TRAP_ENTRY;
        true
    }

    /// Взять прерывание с наивысшим приоритетом (MEI > MSI > MTI), если оно
    /// ожидает, разрешено в `mie` и глобально (`mstatus.MIE`). Возвращает `true`,
    /// если прерывание взято.
    pub fn take_interrupt(&mut self, clk: &mut Clock) -> bool {
        if self.csr.mstatus & MSTATUS_MIE == 0 {
            return false;
        }
        let pending = self.csr.mip & self.csr.mie;
        if pending == 0 {
            return false;
        }
        let code = if pending & MIP_MEIP != 0 {
            cause::M_EXTERNAL_INT
        } else if pending & MIP_MSIP != 0 {
            cause::M_SOFTWARE_INT
        } else {
            cause::M_TIMER_INT
        };
        self.sleeping = false;
        self.interrupts_taken[code as usize] += 1;
        let epc = self.pc;
        self.enter_trap(cause::INTERRUPT | code, 0, epc);
        clk.cycle += timing::TRAP_ENTRY;
        true
    }

    /// Чтение CSR. `None` — такого CSR нет (недопустимая инструкция).
    fn csr_read(&self, n: u32, clk: &Clock, bus: &Bus) -> Option<u32> {
        let cyc = clk.cycle.wrapping_add(self.csr.mcycle_offset);
        let ins = clk.instret.wrapping_add(self.csr.minstret_offset);
        let mtime = bus.mtime(clk.cycle);
        Some(match n {
            0x300 => self.csr.mstatus | MSTATUS_MPP,
            0x301 => MISA,
            0x304 => self.csr.mie,
            0x305 => self.csr.mtvec,
            0x310 => 0, // mstatush: MBE = 0 (little-endian)
            0x320 => 0, // mcountinhibit: счётчики не останавливаются
            0x340 => self.csr.mscratch,
            0x341 => self.csr.mepc,
            0x342 => self.csr.mcause,
            0x343 => self.csr.mtval,
            0x344 => self.csr.mip,
            0xb00 | 0xc00 => cyc as u32,
            0xb80 | 0xc80 => (cyc >> 32) as u32,
            0xb02 | 0xc02 => ins as u32,
            0xb82 | 0xc82 => (ins >> 32) as u32,
            0xc01 => mtime as u32,
            0xc81 => (mtime >> 32) as u32,
            0xf11..=0xf14 => 0, // mvendorid, marchid, mimpid, mhartid = 0
            _ => return None,
        })
    }

    /// Запись CSR. `false` — CSR нет или он только для чтения.
    fn csr_write(&mut self, n: u32, v: u32, clk: &Clock) -> bool {
        // Биты 11:10 = 11 — CSR только для чтения (Privileged ISA, 2.1).
        if n >> 10 == 3 {
            return false;
        }
        match n {
            0x300 => self.csr.mstatus = v & (MSTATUS_MIE | MSTATUS_MPIE),
            0x301 | 0x310 | 0x320 | 0x344 => {} // WARL: запись игнорируется
            0x304 => self.csr.mie = v & (MIP_MSIP | MIP_MTIP | MIP_MEIP),
            0x305 => {
                // MODE: 0 — прямой, 1 — векторный; 2, 3 зарезервированы → прямой.
                let mode = if v & 3 == 1 { 1 } else { 0 };
                self.csr.mtvec = (v & !3) | mode;
            }
            0x340 => self.csr.mscratch = v,
            0x341 => self.csr.mepc = v & !1,
            0x342 => self.csr.mcause = v,
            0x343 => self.csr.mtval = v,
            0xb00 | 0xb80 => {
                let cur = clk.cycle.wrapping_add(self.csr.mcycle_offset);
                let new = if n == 0xb00 {
                    (cur & !0xffff_ffff) | v as u64
                } else {
                    (cur & 0xffff_ffff) | ((v as u64) << 32)
                };
                self.csr.mcycle_offset = new.wrapping_sub(clk.cycle);
            }
            0xb02 | 0xb82 => {
                let cur = clk.instret.wrapping_add(self.csr.minstret_offset);
                let new = if n == 0xb02 {
                    (cur & !0xffff_ffff) | v as u64
                } else {
                    (cur & 0xffff_ffff) | ((v as u64) << 32)
                };
                self.csr.minstret_offset = new.wrapping_sub(clk.instret);
            }
            _ => return false,
        }
        true
    }

    /// Исполняет инструкции, пока `cycle < budget` и не случилось событие,
    /// требующее пересчёта прерываний. Прерывания внутри пачки не проверяются:
    /// бюджет заканчивается ровно на ближайшем событии таймера/сторожа, а всё, что
    /// может изменить линии прерываний (запись CSR, mret, запись в CLINT/INTC/
    /// SYSCTRL/WDT, снятие байта/пакета из FIFO), прерывает пачку.
    pub fn run_batch(&mut self, bus: &mut Bus, clk: &mut Clock, budget: u64) -> Exit {
        // Кэш передаётся отдельным срезом: компилятор тогда знает, что записи в
        // регистры ядра его не меняют, и держит указатель и длину в регистрах хоста.
        let icache = std::mem::take(&mut self.icache);
        let exit = self.run_batch_inner(&icache, bus, clk, budget);
        self.icache = icache;
        exit
    }

    fn run_batch_inner(
        &mut self,
        icache: &[Decoded],
        bus: &mut Bus,
        clk: &mut Clock,
        budget: u64,
    ) -> Exit {
        use Op::*;
        // Горячие счётчики — в локальных переменных; в `self`/`clk` они
        // записываются перед любым вызовом, которому нужны, и на выходе.
        let mut pc = self.pc;
        let mut cycle = clk.cycle;
        let mut instret = clk.instret;

        macro_rules! sync {
            () => {
                self.pc = pc;
                clk.cycle = cycle;
                clk.instret = instret;
            };
        }
        // Исключение по инструкции `pc` (уже потрачено `$cyc` тактов).
        macro_rules! trap {
            ($code:expr, $tval:expr, $cyc:expr) => {{
                cycle += $cyc;
                sync!();
                if !self.exception($code, $tval, pc, clk) {
                    return Exit::Fault;
                }
                pc = self.pc;
                cycle = clk.cycle;
                continue;
            }};
        }

        while cycle < budget {
            let idx = (pc.wrapping_sub(FLASH_BASE) >> 1) as usize;
            let d = match icache.get(idx) {
                Some(d) => *d,
                None => match self.fetch_decode(bus, pc) {
                    Ok(d) => d,
                    Err(()) => trap!(cause::INSTR_ACCESS_FAULT, pc, 0),
                },
            };
            let a = self.get(d.rs1);
            let b = self.get(d.rs2);
            let imm = d.imm;
            let rd = d.rd;
            let next = pc.wrapping_add(d.len as u32);

            match d.op {
                Li => self.set(rd, imm),
                Addi => self.set(rd, a.wrapping_add(imm)),
                Slti => self.set(rd, ((a as i32) < (imm as i32)) as u32),
                Sltiu => self.set(rd, (a < imm) as u32),
                Xori => self.set(rd, a ^ imm),
                Ori => self.set(rd, a | imm),
                Andi => self.set(rd, a & imm),
                Slli => self.set(rd, a << (imm & 31)),
                Srli => self.set(rd, a >> (imm & 31)),
                Srai => self.set(rd, ((a as i32) >> (imm & 31)) as u32),
                Add => self.set(rd, a.wrapping_add(b)),
                Sub => self.set(rd, a.wrapping_sub(b)),
                Sll => self.set(rd, a << (b & 31)),
                Slt => self.set(rd, ((a as i32) < (b as i32)) as u32),
                Sltu => self.set(rd, (a < b) as u32),
                Xor => self.set(rd, a ^ b),
                Srl => self.set(rd, a >> (b & 31)),
                Sra => self.set(rd, ((a as i32) >> (b & 31)) as u32),
                Or => self.set(rd, a | b),
                And => self.set(rd, a & b),
                Jal => {
                    self.set(rd, next);
                    pc = imm;
                    cycle += 1 + timing::JUMP_EXTRA;
                    instret += 1;
                    continue;
                }
                Jalr => {
                    let t = a.wrapping_add(imm) & !1;
                    self.set(rd, next);
                    pc = t;
                    cycle += 1 + timing::JUMP_EXTRA;
                    instret += 1;
                    continue;
                }
                Beq | Bne | Blt | Bge | Bltu | Bgeu => {
                    let taken = match d.op {
                        Beq => a == b,
                        Bne => a != b,
                        Blt => (a as i32) < (b as i32),
                        Bge => (a as i32) >= (b as i32),
                        Bltu => a < b,
                        _ => a >= b,
                    };
                    instret += 1;
                    if taken {
                        pc = imm;
                        cycle += 1 + timing::BRANCH_TAKEN_EXTRA;
                    } else {
                        pc = next;
                        cycle += 1;
                    }
                    continue;
                }
                Lw => {
                    let addr = a.wrapping_add(imm);
                    if addr & 3 != 0 {
                        trap!(cause::LOAD_MISALIGNED, addr, 1 + timing::LOAD_EXTRA);
                    }
                    let off = addr.wrapping_sub(RAM_BASE) as usize;
                    if off <= RAM_SIZE - 4 {
                        let m = &bus.ram[off..off + 4];
                        self.set(rd, u32::from_le_bytes([m[0], m[1], m[2], m[3]]));
                    } else {
                        let v = match bus.read(addr, 4, cycle) {
                            Some(v) => v,
                            None => trap!(cause::LOAD_ACCESS_FAULT, addr, 1 + timing::LOAD_EXTRA),
                        };
                        self.set(rd, v);
                        if is_mmio(addr) {
                            cycle += timing::MMIO_EXTRA;
                        }
                    }
                    pc = next;
                    cycle += 1 + timing::LOAD_EXTRA;
                    instret += 1;
                    if bus.dirty {
                        break;
                    }
                    continue;
                }
                Lb | Lbu | Lh | Lhu => {
                    let addr = a.wrapping_add(imm);
                    let half = matches!(d.op, Lh | Lhu);
                    if half && addr & 1 != 0 {
                        trap!(cause::LOAD_MISALIGNED, addr, 1 + timing::LOAD_EXTRA);
                    }
                    let off = addr.wrapping_sub(RAM_BASE) as usize;
                    let raw = if off <= RAM_SIZE - 2 {
                        if half {
                            u16::from_le_bytes([bus.ram[off], bus.ram[off + 1]]) as u32
                        } else {
                            bus.ram[off] as u32
                        }
                    } else {
                        let size = if half { 2 } else { 1 };
                        if is_mmio(addr) {
                            cycle += timing::MMIO_EXTRA;
                        }
                        match bus.read(addr, size, cycle) {
                            Some(v) => v,
                            None => trap!(cause::LOAD_ACCESS_FAULT, addr, 1 + timing::LOAD_EXTRA),
                        }
                    };
                    let v = match d.op {
                        Lb => raw as u8 as i8 as i32 as u32,
                        Lh => raw as u16 as i16 as i32 as u32,
                        _ => raw,
                    };
                    self.set(rd, v);
                    pc = next;
                    cycle += 1 + timing::LOAD_EXTRA;
                    instret += 1;
                    if bus.dirty {
                        break;
                    }
                    continue;
                }
                Sw | Sh | Sb => {
                    let addr = a.wrapping_add(imm);
                    let size: u32 = match d.op {
                        Sw => 4,
                        Sh => 2,
                        _ => 1,
                    };
                    if addr & (size - 1) != 0 {
                        trap!(cause::STORE_MISALIGNED, addr, 1);
                    }
                    let off = addr.wrapping_sub(RAM_BASE) as usize;
                    if off <= RAM_SIZE - 4 {
                        match size {
                            4 => bus.ram[off..off + 4].copy_from_slice(&b.to_le_bytes()),
                            2 => bus.ram[off..off + 2].copy_from_slice(&(b as u16).to_le_bytes()),
                            _ => bus.ram[off] = b as u8,
                        }
                    } else {
                        if !bus.write(addr, size, b, cycle) {
                            trap!(cause::STORE_ACCESS_FAULT, addr, 1);
                        }
                        if is_mmio(addr) {
                            cycle += timing::MMIO_EXTRA;
                        }
                    }
                    pc = next;
                    cycle += 1;
                    instret += 1;
                    if bus.dirty {
                        break;
                    }
                    continue;
                }
                Mul | Mulh | Mulhsu | Mulhu => {
                    let v = match d.op {
                        Mul => a.wrapping_mul(b),
                        Mulh => (((a as i32 as i64) * (b as i32 as i64)) >> 32) as u32,
                        Mulhsu => ((a as i32 as i64).wrapping_mul(b as u64 as i64) >> 32) as u32,
                        _ => (((a as u64) * (b as u64)) >> 32) as u32,
                    };
                    self.set(rd, v);
                    cycle += timing::MUL_EXTRA;
                }
                Div | Divu | Rem | Remu => {
                    // Деление на 0: частное — все единицы, остаток — делимое;
                    // переполнение INT_MIN/−1: частное INT_MIN, остаток 0 (Vol. I, 7.2).
                    let v = match d.op {
                        Div if b == 0 => u32::MAX,
                        Div => (a as i32).wrapping_div(b as i32) as u32,
                        Divu => a.checked_div(b).unwrap_or(u32::MAX),
                        Rem if b == 0 => a,
                        Rem => (a as i32).wrapping_rem(b as i32) as u32,
                        _ => a.checked_rem(b).unwrap_or(a),
                    };
                    self.set(rd, v);
                    cycle += timing::DIV_EXTRA;
                }
                Fence | FenceI => {}
                Ecall => trap!(cause::ECALL_M, 0, 1),
                Ebreak => {
                    // Остановка в отладчик: pc остаётся на ebreak.
                    cycle += 1;
                    sync!();
                    return Exit::Breakpoint;
                }
                Mret => {
                    let mpie = self.csr.mstatus & MSTATUS_MPIE != 0;
                    self.csr.mstatus = MSTATUS_MPIE | if mpie { MSTATUS_MIE } else { 0 };
                    self.in_trap = false;
                    pc = self.csr.mepc;
                    cycle += 1 + timing::MRET_EXTRA;
                    instret += 1;
                    sync!();
                    return Exit::Reeval;
                }
                Wfi => {
                    pc = next;
                    cycle += 1;
                    instret += 1;
                    if self.csr.mip & self.csr.mie != 0 {
                        continue; // уже есть ожидающее прерывание — wfi как nop
                    }
                    self.sleeping = true;
                    sync!();
                    return Exit::Sleep;
                }
                Csrrw | Csrrs | Csrrc | Csrrwi | Csrrsi | Csrrci => {
                    let n = imm & 0xfff;
                    // rs1 хранит номер регистра или 5-битную константу zimm.
                    let src = d.rs1 & 31;
                    let operand = match d.op {
                        Csrrw | Csrrs | Csrrc => a,
                        _ => src as u32,
                    };
                    sync!();
                    let old = match self.csr_read(n, clk, bus) {
                        Some(v) => v,
                        None => trap!(cause::ILLEGAL_INSTRUCTION, self.raw_bits(bus, pc), 1),
                    };
                    // csrrs/csrrc с rs1 = x0 (или zimm = 0) не пишут — и не ловят RO.
                    let write = match d.op {
                        Csrrw | Csrrwi => Some(operand),
                        Csrrs | Csrrsi if src != 0 => Some(old | operand),
                        Csrrc | Csrrci if src != 0 => Some(old & !operand),
                        _ => None,
                    };
                    if let Some(v) = write
                        && !self.csr_write(n, v, clk)
                    {
                        trap!(cause::ILLEGAL_INSTRUCTION, self.raw_bits(bus, pc), 1);
                    }
                    self.set(rd, old);
                    pc = next;
                    cycle += 1 + timing::CSR_EXTRA;
                    instret += 1;
                    if write.is_some() {
                        sync!();
                        return Exit::Reeval;
                    }
                    continue;
                }
                Illegal => trap!(cause::ILLEGAL_INSTRUCTION, imm, 1),
            }
            pc = next;
            cycle += 1;
            instret += 1;
        }
        sync!();
        if bus.dirty {
            Exit::Reeval
        } else {
            Exit::Budget
        }
    }

    /// Биты инструкции по адресу (для `mtval` недопустимой инструкции).
    fn raw_bits(&self, bus: &Bus, pc: u32) -> u32 {
        let lo = bus.fetch16(pc).unwrap_or(0) as u32;
        if lo & 3 == 3 {
            lo | ((bus.fetch16(pc.wrapping_add(2)).unwrap_or(0) as u32) << 16)
        } else {
            lo
        }
    }
}
