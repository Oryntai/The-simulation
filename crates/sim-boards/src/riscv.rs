//! RV32IMC: Unprivileged ISA §§2, 13, 16, 27 и Privileged ISA §3.
//! Декодирование разделено с sim-mcu, карта памяти и прерывания принадлежат ESP32-C3.

use crate::{BoardFault, memory::Bus};
use sim_mcu::cpu::{Op, decode};

#[derive(Clone, Default)]
pub(crate) struct Riscv {
    r: [u32; 32],
    pub pc: u32,
    status: u32,
    vector: u32,
    epc: u32,
    cause: u32,
    scratch: u32,
    instret: u64,
    sleeping: bool,
}

impl Riscv {
    pub fn boot(bus: &Bus, entry: u32) -> Result<Self, BoardFault> {
        if entry & 1 != 0 {
            return Err(BoardFault::BadImage(
                "не выровнена точка входа RISC-V".into(),
            ));
        }
        bus.fetch(entry, 2)?;
        Ok(Self {
            pc: entry,
            ..Self::default()
        })
    }

    fn trap(&mut self, code: u32, irq: bool) -> Result<(), BoardFault> {
        if self.vector == 0 {
            return Err(BoardFault::InvalidState {
                pc: self.pc,
                target: 0,
                why: "не задан mtvec",
            });
        }
        self.epc = self.pc;
        self.cause = (u32::from(irq) << 31) | code;
        self.status = (self.status & 8) << 4;
        self.pc = (self.vector & !3).wrapping_add(if irq && self.vector & 3 == 1 {
            4 * code
        } else {
            0
        });
        self.sleeping = false;
        Ok(())
    }

    pub fn step(&mut self, bus: &mut Bus) -> Result<u64, BoardFault> {
        use Op::*;
        if self.status & 8 != 0
            && let Some(irq) = bus.hw.esp_irq()?
        {
            self.trap(irq, true)?;
            return Ok(1);
        }
        if self.sleeping {
            return Ok(1);
        }

        let pc = self.pc;
        let low = bus.fetch(pc, 2)?;
        let raw = if low & 3 == 3 {
            low | bus.fetch(pc.wrapping_add(2), 2)? << 16
        } else {
            low
        };
        let d = decode(raw, pc);
        let unknown = || BoardFault::UnknownInstruction {
            pc,
            raw,
            isa: "RV32IMC",
        };
        let next = pc.wrapping_add(u32::from(d.len));
        let a = self.r[d.rs1 as usize];
        let b = self.r[d.rs2 as usize];
        let i = d.imm;
        let retired = self.instret;
        self.instret = retired.wrapping_add(1);
        // Оценка времени — один такт на инструкцию; задержки шины не моделируются.
        self.pc = next;
        let value = match d.op {
            Li => i,
            Jal | Jalr => {
                self.pc = if d.op == Jal {
                    i
                } else {
                    a.wrapping_add(i) & !1
                };
                next
            }
            Beq | Bne | Blt | Bge | Bltu | Bgeu => {
                let take = match d.op {
                    Beq => a == b,
                    Bne => a != b,
                    Blt => (a as i32) < b as i32,
                    Bge => (a as i32) >= b as i32,
                    Bltu => a < b,
                    _ => a >= b,
                };
                if take {
                    self.pc = i;
                }
                return Ok(1);
            }
            Lb | Lh | Lw | Lbu | Lhu => {
                let size = match d.op {
                    Lb | Lbu => 1,
                    Lh | Lhu => 2,
                    _ => 4,
                };
                let v = bus.read(pc, a.wrapping_add(i), size)?;
                match d.op {
                    Lb => v as i8 as i32 as u32,
                    Lh => v as i16 as i32 as u32,
                    _ => v,
                }
            }
            Sb | Sh | Sw => {
                let size = match d.op {
                    Sb => 1,
                    Sh => 2,
                    _ => 4,
                };
                bus.write(pc, a.wrapping_add(i), size, b)?;
                return Ok(1);
            }
            Add => a.wrapping_add(b),
            Addi => a.wrapping_add(i),
            Sub => a.wrapping_sub(b),
            Slt => u32::from((a as i32) < b as i32),
            Slti => u32::from((a as i32) < i as i32),
            Sltu => u32::from(a < b),
            Sltiu => u32::from(a < i),
            Xor => a ^ b,
            Xori => a ^ i,
            Or => a | b,
            Ori => a | i,
            And => a & b,
            Andi => a & i,
            Sll => a << (b & 31),
            Slli => a << (i & 31),
            Srl => a >> (b & 31),
            Srli => a >> (i & 31),
            Sra => ((a as i32) >> (b & 31)) as u32,
            Srai => ((a as i32) >> (i & 31)) as u32,
            Mul => a.wrapping_mul(b),
            Mulh => ((i64::from(a as i32) * i64::from(b as i32)) >> 32) as u32,
            Mulhu => ((u64::from(a) * u64::from(b)) >> 32) as u32,
            Mulhsu => ((i64::from(a as i32) * i64::from(b)) >> 32) as u32,
            Div => {
                if b == 0 {
                    u32::MAX
                } else {
                    (a as i32).wrapping_div(b as i32) as u32
                }
            }
            Divu => a.checked_div(b).unwrap_or(u32::MAX),
            Rem => {
                if b == 0 {
                    a
                } else {
                    (a as i32).wrapping_rem(b as i32) as u32
                }
            }
            Remu => a.checked_rem(b).unwrap_or(a),
            Fence | FenceI => return Ok(1),
            Wfi => {
                self.sleeping = true;
                return Ok(1);
            }
            Ebreak => return Err(BoardFault::Breakpoint { pc }),
            Ecall => {
                self.pc = pc;
                self.instret = retired;
                self.trap(11, false)?;
                return Ok(1);
            }
            Mret => {
                self.pc = self.epc;
                self.status = 0x80 | ((self.status & 0x80) >> 4);
                return Ok(1);
            }
            Csrrw | Csrrs | Csrrc | Csrrwi | Csrrsi | Csrrci => {
                let src = if matches!(d.op, Csrrwi | Csrrsi | Csrrci) {
                    u32::from(d.rs1)
                } else {
                    a
                };
                let old = match i {
                    0x300 => self.status | (3 << 11),
                    0x301 => sim_mcu::cpu::MISA,
                    0x305 => self.vector,
                    0x340 => self.scratch,
                    0x341 => self.epc,
                    0x342 => self.cause,
                    0x343 | 0xf14 => 0,
                    0xb00 | 0xc00 => bus.hw.cycles as u32,
                    0xb80 | 0xc80 => (bus.hw.cycles >> 32) as u32,
                    0xb02 | 0xc02 => retired as u32,
                    0xb82 | 0xc82 => (retired >> 32) as u32,
                    _ => return Err(unknown()),
                };
                let new = match d.op {
                    Csrrw | Csrrwi => Some(src),
                    Csrrs | Csrrsi if d.rs1 != 0 => Some(old | src),
                    Csrrc | Csrrci if d.rs1 != 0 => Some(old & !src),
                    _ => None,
                };
                if let Some(new) = new {
                    match i {
                        0x300 if new & !0x1888 == 0 => self.status = new & 0x88,
                        0x305 if new & 3 <= 1 && (new & 3 == 0 || new & 0xff == 1) => {
                            self.vector = new
                        }
                        0x340 => self.scratch = new,
                        0x341 => self.epc = new & !1,
                        0x342 => self.cause = new,
                        _ => return Err(unknown()),
                    }
                }
                old
            }
            Illegal => return Err(unknown()),
        };
        if d.rd != 0 {
            self.r[d.rd as usize] = value;
        }
        Ok(1)
    }
}
