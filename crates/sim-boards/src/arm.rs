//! Thumb: Armv6-M Architecture Reference Manual DDI0419 и ST PM0056 для Armv7-M.
//! Время инструкций — оценка, без ожиданий памяти и конвейера.

use crate::{BoardFault, BoardId, memory::Bus};

const SP: usize = 13;
const LR: usize = 14;

#[derive(Clone, Default)]
pub(crate) struct Arm {
    r: [u32; 15],
    pub pc: u32,
    n: bool,
    z: bool,
    c: bool,
    v: bool,
    masked: bool,
    active: u16,
    sleeping: bool,
    it: u8,
}

impl Arm {
    pub fn boot(bus: &mut Bus, base: u32) -> Result<Self, BoardFault> {
        let mut cpu = Self::default();
        let sp = bus.fetch(base, 4)?;
        let limit = if bus.id == BoardId::Rp2040 {
            264 << 10
        } else {
            20 << 10
        };
        if sp & 7 != 0 || !(0x2000_0008..=0x2000_0000 + limit).contains(&sp) {
            return Err(BoardFault::BadImage(
                "начальный MSP вне SRAM или не выровнен".into(),
            ));
        }
        cpu.r[SP] = sp;
        cpu.branch(bus.fetch(base + 4, 4)?)?;
        bus.fetch(cpu.pc, 2)?;
        Ok(cpu)
    }

    fn branch(&mut self, target: u32) -> Result<(), BoardFault> {
        if target & 1 == 0 {
            return Err(BoardFault::InvalidState {
                pc: self.pc,
                target,
                why: "вектор или BX без бита Thumb",
            });
        }
        self.pc = target & !1;
        Ok(())
    }

    fn nz(&mut self, value: u32) -> u32 {
        self.n = value >> 31 != 0;
        self.z = value == 0;
        value
    }

    fn add(&mut self, a: u32, b: u32, carry: bool, flags: bool) -> u32 {
        let sum = u64::from(a) + u64::from(b) + u64::from(carry);
        let signed = i64::from(a as i32) + i64::from(b as i32) + i64::from(carry);
        let value = sum as u32;
        if flags {
            self.nz(value);
            self.c = sum >> 32 != 0;
            self.v = signed != i64::from(value as i32);
        }
        value
    }

    fn shift_carry(&self, a: u32, typ: u16, amount: u32) -> (u32, bool) {
        match (typ, amount) {
            (_, 0) => (a, self.c),
            (0, 1..=32) => (
                a.checked_shl(amount).unwrap_or(0),
                a >> (32 - amount) & 1 != 0,
            ),
            (1, 1..=32) => (
                a.checked_shr(amount).unwrap_or(0),
                a >> (amount - 1) & 1 != 0,
            ),
            (0 | 1, _) => (0, false),
            (2, _) => {
                let shift = amount.min(32);
                (
                    ((a as i32) >> shift.min(31)) as u32,
                    a >> (shift - 1) & 1 != 0,
                )
            }
            _ => {
                let value = a.rotate_right(amount);
                (value, value >> 31 != 0)
            }
        }
    }

    fn shift(&mut self, a: u32, typ: u16, amount: u32, flags: bool) -> u32 {
        let (value, carry) = self.shift_carry(a, typ, amount);
        if flags {
            self.nz(value);
            self.c = carry;
        }
        value
    }

    fn condition(&self, code: u8) -> bool {
        let yes = match code >> 1 {
            0 => self.z,
            1 => self.c,
            2 => self.n,
            3 => self.v,
            4 => self.c && !self.z,
            5 => self.n == self.v,
            6 => !self.z && self.n == self.v,
            _ => true,
        };
        yes != (code & 1 != 0)
    }

    fn reg(&self, index: usize, pc: u32) -> u32 {
        if index == 15 {
            pc.wrapping_add(4)
        } else {
            self.r[index]
        }
    }

    fn psr(&self) -> u32 {
        (u32::from(self.n) << 31)
            | (u32::from(self.z) << 30)
            | (u32::from(self.c) << 29)
            | (u32::from(self.v) << 28)
            | (1 << 24)
            | u32::from(self.active)
            | (u32::from(self.it & 3) << 25)
            | (u32::from(self.it & 0xfc) << 8)
    }

    fn take_exception(&mut self, bus: &mut Bus, exception: u16) -> Result<(), BoardFault> {
        let old_sp = self.r[SP];
        let sp = (old_sp & !7).wrapping_sub(32);
        let frame = [
            self.r[0],
            self.r[1],
            self.r[2],
            self.r[3],
            self.r[12],
            self.r[LR],
            self.pc,
            self.psr() | (u32::from(old_sp & 4 != 0) << 9),
        ];
        for (i, x) in frame.into_iter().enumerate() {
            bus.write(self.pc, sp.wrapping_add(i as u32 * 4), 4, x)?;
        }
        self.r[SP] = sp;
        // EXC_RETURN с MSP: Thread mode (F9) или прерванный Handler mode (F1).
        self.r[LR] = if self.active == 0 {
            0xffff_fff9
        } else {
            0xffff_fff1
        };
        self.active = exception;
        bus.hw.arm_active = exception;
        bus.hw.arm_active_mask |= 1 << exception;
        self.it = 0;
        self.sleeping = false;
        let vector = bus.hw.vector_base.wrapping_add(u32::from(exception) * 4);
        self.branch(bus.read(self.pc, vector, 4)?)
    }

    fn bx(&mut self, bus: &mut Bus, target: u32) -> Result<(), BoardFault> {
        if matches!(target, 0xffff_fff9 | 0xffff_fff1) && self.active != 0 {
            let sp = self.r[SP];
            let mut frame = [0; 8];
            for (i, x) in frame.iter_mut().enumerate() {
                *x = bus.read(self.pc, sp.wrapping_add(i as u32 * 4), 4)?;
            }
            let psr = frame[7];
            let next_active = (psr & 0x1ff) as u16;
            let remaining = bus.hw.arm_active_mask & !(1 << self.active);
            let valid_mode = if target == 0xffff_fff9 {
                next_active == 0 && remaining == 0
            } else {
                next_active < 64 && remaining & (1 << next_active) != 0
            };
            if psr & (1 << 24) == 0 || frame[6] & 1 != 0 || !valid_mode {
                return Err(BoardFault::Lockup {
                    pc: self.pc,
                    why: "повреждён кадр исключения".into(),
                });
            }
            self.r[..4].copy_from_slice(&frame[..4]);
            self.r[12] = frame[4];
            self.r[LR] = frame[5];
            self.pc = frame[6];
            self.r[SP] = sp.wrapping_add(32 + ((psr >> 9 & 1) * 4));
            self.n = psr >> 31 != 0;
            self.z = psr >> 30 & 1 != 0;
            self.c = psr >> 29 & 1 != 0;
            self.v = psr >> 28 & 1 != 0;
            self.it = ((psr >> 25 & 3) | (psr >> 8 & 0xfc)) as u8;
            self.active = next_active;
            bus.hw.arm_active = next_active;
            bus.hw.arm_active_mask = remaining;
            return Ok(());
        }
        self.branch(target)
    }

    pub fn step(&mut self, bus: &mut Bus) -> Result<u64, BoardFault> {
        if let Some(exception) = bus.hw.arm_exception(self.masked) {
            self.take_exception(bus, exception)?;
            // PM0056 / Cortex-M0+ TRM, без ожиданий памяти.
            return Ok(if bus.id == BoardId::Rp2040 { 15 } else { 12 });
        }
        if self.sleeping {
            return Ok(1);
        }
        let pc = self.pc;
        let m3 = bus.id == BoardId::Stm32f103c8;
        let h = bus.fetch(pc, 2)? as u16;
        let wide = h >> 11 >= 0b11101;
        let next = pc.wrapping_add(if wide { 4 } else { 2 });
        let lo = if wide {
            bus.fetch(pc.wrapping_add(2), 2)? as u16
        } else {
            0
        };
        let unknown = || BoardFault::UnknownInstruction {
            pc,
            raw: if wide {
                (u32::from(h) << 16) | u32::from(lo)
            } else {
                u32::from(h)
            },
            isa: if m3 {
                "Thumb-2 (ARMv7-M)"
            } else {
                "Thumb (ARMv6-M)"
            },
        };
        let in_it = self.it != 0;
        if in_it {
            let cond = self.it >> 4;
            self.it = if self.it & 7 == 0 {
                0
            } else {
                (self.it & 0xe0) | ((self.it << 1) & 0x1f)
            };
            if !self.condition(cond) {
                // Неизвестная инструкция, условие которой ложно, архитектурно не исполняется.
                self.pc = next;
                return Ok(1);
            }
        }
        self.pc = next;
        if wide {
            return self
                .wide(bus, pc, h, lo, m3)
                .and_then(|cycles| cycles.ok_or_else(unknown));
        }
        let d = (h & 7) as usize;
        let s = (h >> 3 & 7) as usize;
        let a = (h >> 6 & 7) as usize;
        let r8 = (h >> 8 & 7) as usize;
        let imm = u32::from(h & 0xff);
        let mut cycles = 1;
        match h {
            0x0000..=0x17ff => {
                let typ = h >> 11;
                let amount = if typ != 0 && h >> 6 & 31 == 0 {
                    32
                } else {
                    u32::from(h >> 6 & 31)
                };
                self.r[d] = self.shift(self.r[s], typ, amount, !in_it);
            }
            0x1800..=0x1fff => {
                let rhs = if h & 0x400 != 0 { a as u32 } else { self.r[a] };
                let sub = h & 0x200 != 0;
                self.r[d] = self.add(self.r[s], if sub { !rhs } else { rhs }, sub, !in_it);
            }
            0x2000..=0x3fff => {
                let old = self.r[r8];
                match h >> 11 & 3 {
                    0 => {
                        self.r[r8] = imm;
                        if !in_it {
                            self.nz(imm);
                        }
                    }
                    1 => {
                        self.add(old, !imm, true, true);
                    }
                    2 => self.r[r8] = self.add(old, imm, false, !in_it),
                    _ => self.r[r8] = self.add(old, !imm, true, !in_it),
                }
            }
            0x4000..=0x43ff => {
                let left = self.r[d];
                let right = self.r[s];
                let op = h >> 6 & 15;
                let value = match op {
                    0 => left & right,
                    1 => left ^ right,
                    2..=4 => self.shift(left, op - 2, right & 255, !in_it),
                    5 => self.add(left, right, self.c, !in_it),
                    6 => self.add(left, !right, self.c, !in_it),
                    7 => self.shift(left, 3, right & 255, !in_it),
                    8 => {
                        self.nz(left & right);
                        return Ok(1);
                    }
                    9 => self.add(0, !right, true, !in_it),
                    10 => {
                        self.add(left, !right, true, true);
                        return Ok(1);
                    }
                    11 => {
                        self.add(left, right, false, true);
                        return Ok(1);
                    }
                    12 => left | right,
                    13 => left.wrapping_mul(right),
                    14 => left & !right,
                    _ => !right,
                };
                if !in_it && matches!(op, 0 | 1 | 12..=15) {
                    self.nz(value);
                }
                self.r[d] = value;
            }
            0x4400..=0x47ff => {
                let rd = usize::from((h & 7) | (h >> 4 & 8));
                let rm = usize::from(h >> 3 & 15);
                let right = self.reg(rm, pc);
                match h >> 8 & 3 {
                    0 | 2 => {
                        let v = if h & 0x200 == 0 {
                            self.reg(rd, pc).wrapping_add(right)
                        } else {
                            right
                        };
                        if rd == 15 {
                            self.pc = v & !1;
                            cycles = 3;
                        } else {
                            self.r[rd] = if rd == SP { v & !3 } else { v };
                        }
                    }
                    1 => {
                        if rd == 15 || rm == 15 {
                            return Err(unknown());
                        }
                        self.add(self.r[rd], !right, true, true);
                    }
                    _ => {
                        if h & 7 != 0 || rm == 15 {
                            return Err(unknown());
                        }
                        if h & 0x80 != 0 {
                            self.r[LR] = next | 1;
                        }
                        self.bx(bus, right)?;
                        cycles = 3;
                    }
                }
            }
            0x4800..=0x4fff => {
                self.r[r8] = bus.read(pc, (pc.wrapping_add(4) & !3).wrapping_add(imm * 4), 4)?;
                cycles = 2;
            }
            0x5000..=0x9fff => {
                let (address, size, load, signed, rd) = if h < 0x6000 {
                    let op = h >> 9 & 7;
                    (
                        self.r[s].wrapping_add(self.r[a]),
                        [4, 2, 1, 1, 4, 2, 1, 2][op as usize],
                        op >= 3,
                        matches!(op, 3 | 7),
                        d,
                    )
                } else if h < 0x9000 {
                    let size = if h >= 0x8000 {
                        2
                    } else if h & 0x1000 == 0 {
                        4
                    } else {
                        1
                    };
                    (
                        self.r[s].wrapping_add(u32::from(h >> 6 & 31) * u32::from(size)),
                        size,
                        h & 0x800 != 0,
                        false,
                        d,
                    )
                } else {
                    (
                        self.r[SP].wrapping_add(imm * 4),
                        4,
                        h & 0x800 != 0,
                        false,
                        r8,
                    )
                };
                if load {
                    let v = bus.read(pc, address, size)?;
                    self.r[rd] = if signed && size == 1 {
                        v as i8 as i32 as u32
                    } else if signed {
                        v as i16 as i32 as u32
                    } else {
                        v
                    };
                } else {
                    bus.write(pc, address, size, self.r[rd])?;
                }
                cycles = 2;
            }
            0xa000..=0xafff => {
                let base = if h & 0x800 == 0 {
                    pc.wrapping_add(4) & !3
                } else {
                    self.r[SP]
                };
                self.r[r8] = base.wrapping_add(imm * 4);
            }
            0xb000..=0xb0ff => {
                let delta = u32::from(h & 127) * 4;
                self.r[SP] = self.r[SP].wrapping_add(if h & 128 == 0 {
                    delta
                } else {
                    delta.wrapping_neg()
                });
            }
            _ if m3 && h & 0xf500 == 0xb100 => {
                if (self.r[d] != 0) == (h & 0x800 != 0) {
                    self.pc = pc.wrapping_add(4 + u32::from((h >> 2 & 0x3e) | (h >> 3 & 0x40)));
                    cycles = 3;
                }
            }
            0xb200..=0xb2ff => {
                let value = self.r[s];
                self.r[d] = match h >> 6 & 3 {
                    0 => value as i16 as i32 as u32,
                    1 => value as i8 as i32 as u32,
                    2 => value & 0xffff,
                    _ => value & 0xff,
                };
            }
            0xb400..=0xb5ff | 0xbc00..=0xbdff => {
                let pop = h & 0x800 != 0;
                let extra = h & 0x100 != 0;
                let count = imm.count_ones() + u32::from(extra);
                if count == 0 {
                    return Err(unknown());
                }
                let mut addr = if pop {
                    self.r[SP]
                } else {
                    self.r[SP].wrapping_sub(count * 4)
                };
                if !pop {
                    self.r[SP] = addr;
                }
                for (index, register) in self.r[..8].iter_mut().enumerate() {
                    if imm & (1 << index) == 0 {
                        continue;
                    }
                    if pop {
                        *register = bus.read(pc, addr, 4)?;
                    } else {
                        bus.write(pc, addr, 4, *register)?;
                    }
                    addr = addr.wrapping_add(4);
                }
                if extra {
                    if pop {
                        let target = bus.read(pc, addr, 4)?;
                        self.r[SP] = addr.wrapping_add(4);
                        self.bx(bus, target)?;
                    } else {
                        bus.write(pc, addr, 4, self.r[LR])?;
                    }
                } else if pop {
                    self.r[SP] = addr;
                }
                cycles = 1 + u64::from(count);
            }
            0xb662 => self.masked = false,
            0xb672 => self.masked = true,
            0xba00..=0xbabf => {
                let value = self.r[s];
                self.r[d] = match h >> 6 & 3 {
                    0 => value.swap_bytes(),
                    1 => value.rotate_left(16).swap_bytes(),
                    3 => (value as u16).swap_bytes() as i16 as i32 as u32,
                    _ => return Err(unknown()),
                };
            }
            0xbe00..=0xbeff => return Err(BoardFault::Breakpoint { pc }),
            0xbf00 | 0xbf10 => {}
            0xbf30 => self.sleeping = true,
            0xbf01..=0xbfff if m3 && (h & 15 != 0) && h & 0xe0 != 0xe0 && !in_it => {
                self.it = imm as u8
            }
            0xc000..=0xcfff => {
                if imm == 0
                    || h & 0x800 == 0 && imm & (1 << r8) != 0 && imm.trailing_zeros() as usize != r8
                {
                    return Err(unknown());
                }
                let mut addr = self.r[r8];
                for (index, register) in self.r[..8].iter_mut().enumerate() {
                    if imm & (1 << index) == 0 {
                        continue;
                    }
                    if h & 0x800 != 0 {
                        *register = bus.read(pc, addr, 4)?;
                    } else {
                        bus.write(pc, addr, 4, *register)?;
                    }
                    addr = addr.wrapping_add(4);
                    cycles += 1;
                }
                if h & 0x800 == 0 || imm & (1 << r8) == 0 {
                    self.r[r8] = addr;
                }
            }
            0xd000..=0xddff => {
                if self.condition((h >> 8 & 15) as u8) {
                    self.pc = pc
                        .wrapping_add(4)
                        .wrapping_add((imm as i8 as i32 * 2) as u32);
                    cycles = 3;
                }
            }
            0xdf00..=0xdfff if !self.masked && self.active == 0 => {
                self.take_exception(bus, 11)?;
                cycles = 12;
            }
            0xe000..=0xe7ff => {
                self.pc = pc
                    .wrapping_add(4)
                    .wrapping_add(sign(u32::from(h & 0x7ff) << 1, 12));
                cycles = 3;
            }
            _ => return Err(unknown()),
        }
        Ok(cycles)
    }

    fn wide(
        &mut self,
        bus: &mut Bus,
        pc: u32,
        h: u16,
        l: u16,
        m3: bool,
    ) -> Result<Option<u64>, BoardFault> {
        let rd = usize::from(l >> 8 & 15);
        let rn = usize::from(h & 15);
        let rm = usize::from(l & 15);
        if h == 0xf3bf && matches!(l, 0x8f4f | 0x8f5f | 0x8f6f) {
            return Ok(Some(1));
        } // DSB, DMB, ISB.
        if h & 0xf800 == 0xf000 && (l & 0xd000 == 0xd000 || m3 && l & 0xd000 == 0x9000) {
            let s = u32::from(h >> 10 & 1);
            let i1 = 1 ^ (u32::from(l >> 13 & 1) ^ s);
            let i2 = 1 ^ (u32::from(l >> 11 & 1) ^ s);
            let offset = (s << 24)
                | (i1 << 23)
                | (i2 << 22)
                | (u32::from(h & 0x3ff) << 12)
                | (u32::from(l & 0x7ff) << 1);
            if l & 0x4000 != 0 {
                self.r[LR] = pc.wrapping_add(4) | 1;
            }
            self.pc = pc.wrapping_add(4).wrapping_add(sign(offset, 25));
            return Ok(Some(3));
        }
        // B<cond>.W (T3): J2 и J1 входят в знаковое смещение напрямую.
        let condition = (h >> 6 & 15) as u8;
        if m3 && h & 0xf800 == 0xf000 && l & 0xd000 == 0x8000 && condition < 14 {
            if self.condition(condition) {
                let offset = (u32::from(h >> 10 & 1) << 20)
                    | (u32::from(l >> 11 & 1) << 19)
                    | (u32::from(l >> 13 & 1) << 18)
                    | (u32::from(h & 63) << 12)
                    | (u32::from(l & 0x7ff) << 1);
                self.pc = pc.wrapping_add(4).wrapping_add(sign(offset, 21));
                return Ok(Some(3));
            }
            return Ok(Some(1));
        }
        if h == 0xf3ef && l & 0xf000 == 0x8000 && rd < SP {
            self.r[rd] = match l & 255 {
                0 => self.psr() & 0xf000_0000,
                5 => u32::from(self.active),
                8 => self.r[SP],
                16 => u32::from(self.masked),
                20 => 0,
                _ => return Ok(None),
            };
            return Ok(Some(1));
        }
        if h & 0xfff0 == 0xf380 && l == 0x8810 && rn < SP {
            self.masked = self.r[rn] & 1 != 0;
            return Ok(Some(1));
        }
        if !m3 {
            return Ok(None);
        }
        if h & 0xfbf0 == 0xf240 || h & 0xfbf0 == 0xf2c0 {
            if rd >= SP || l & 0x8000 != 0 {
                return Ok(None);
            }
            let imm = (u32::from(h & 15) << 12)
                | (u32::from(h & 0x400) << 1)
                | (u32::from(l & 0x7000) >> 4)
                | u32::from(l & 255);
            self.r[rd] = if h & 0x80 != 0 {
                (self.r[rd] & 0xffff) | (imm << 16)
            } else {
                imm
            };
            return Ok(Some(1));
        }
        // LDRD/STRD с двумя регистрами, непосредственное смещение times 4.
        if h & 0xfe40 == 0xe840 {
            let index = h & 0x100 != 0;
            let up = h & 0x80 != 0;
            let wb = h & 0x20 != 0;
            let load = h & 0x10 != 0;
            let rt = usize::from(l >> 12);
            if !index && !wb
                || rn == 15 && (!load || wb)
                || rt >= SP
                || rd >= SP
                || load && rt == rd
                || wb && (rn == rt || rn == rd)
            {
                return Ok(None);
            }
            let base = if rn == 15 {
                pc.wrapping_add(4) & !3
            } else {
                self.r[rn]
            };
            let offset = u32::from(l & 255) * 4;
            let changed = if up {
                base.wrapping_add(offset)
            } else {
                base.wrapping_sub(offset)
            };
            let addr = if index { changed } else { base };
            if load {
                self.r[rt] = bus.read(pc, addr, 4)?;
                self.r[rd] = bus.read(pc, addr.wrapping_add(4), 4)?;
            } else {
                bus.write(pc, addr, 4, self.r[rt])?;
                bus.write(pc, addr.wrapping_add(4), 4, self.r[rd])?;
            }
            if wb {
                self.r[rn] = changed;
            }
            return Ok(Some(3));
        }
        // ADDW/SUBW и ADR.W (PM0056, plain binary immediate).
        if matches!(h & 0xfbf0, 0xf200 | 0xf2a0) && l & 0x8000 == 0 {
            if rd >= SP && !(rd == SP && rn == SP) {
                return Ok(None);
            }
            let imm =
                (u32::from(h & 0x400) << 1) | (u32::from(l & 0x7000) >> 4) | u32::from(l & 255);
            let base = if rn == 15 {
                pc.wrapping_add(4) & !3
            } else {
                self.r[rn]
            };
            let v = if h & 0x80 != 0 {
                base.wrapping_sub(imm)
            } else {
                base.wrapping_add(imm)
            };
            self.r[rd] = if rd == SP { v & !3 } else { v };
            return Ok(Some(1));
        }
        if l & 0x8000 == 0 && (h & 0xfa00 == 0xf000 || h & 0xfe00 == 0xea00) {
            let (operand, carry) = if h & 0xfe00 == 0xea00 {
                if rm >= SP {
                    return Ok(None);
                }
                // Thumb-2 Data processing (shifted register), включая RRX.
                let typ = l >> 4 & 3;
                let amount = u32::from((l >> 10 & 28) | (l >> 6 & 3));
                let value = self.r[rm];
                match (typ, amount) {
                    (3, 0) => ((u32::from(self.c) << 31) | (value >> 1), value & 1 != 0),
                    (1 | 2, 0) => self.shift_carry(value, typ, 32),
                    _ => self.shift_carry(value, typ, amount),
                }
            } else {
                // ThumbExpandImm_C, PM0056: модифицированный immediate.
                let field = ((u32::from(h) >> 10 & 1) << 11)
                    | ((u32::from(l) >> 12 & 7) << 8)
                    | u32::from(l & 255);
                let byte = field & 255;
                if field >> 10 == 0 {
                    let value = match field >> 8 & 3 {
                        0 => byte,
                        _ if byte == 0 => return Ok(None),
                        1 => byte * 0x0001_0001,
                        2 => byte * 0x0100_0100,
                        _ => byte * 0x0101_0101,
                    };
                    (value, self.c)
                } else {
                    let value = (128 | (field & 127)).rotate_right(field >> 7);
                    (value, value >> 31 != 0)
                }
            };
            let set = h & 16 != 0;
            let op = h >> 5 & 15;
            let compare = rd == 15 && set && matches!(op, 0 | 4 | 8 | 13);
            let sp_math = rn == SP && matches!(op, 8 | 13) && !compare && !set;
            if !compare && rd >= SP && !(sp_math && rd == SP)
                || rn == SP && !sp_math
                || rn == 15 && !matches!(op, 2 | 3)
            {
                return Ok(None);
            }
            let a = if rn == 15 { 0 } else { self.r[rn] };
            let value = match op {
                0 => a & operand,
                1 => a & !operand,
                2 => a | operand,
                3 => a | !operand,
                4 => a ^ operand,
                8 => self.add(a, operand, false, set),
                10 => self.add(a, operand, self.c, set),
                11 => self.add(a, !operand, self.c, set),
                13 => self.add(a, !operand, true, set),
                14 => self.add(operand, !a, true, set),
                _ => return Ok(None),
            };
            if set && op <= 4 {
                self.nz(value);
                self.c = carry;
            }
            if !compare {
                self.r[rd] = if rd == SP { value & !3 } else { value };
            }
            return Ok(Some(1));
        }
        if matches!(h & 0xfff0, 0xfb90 | 0xfbb0)
            && l & 0xf0f0 == 0xf0f0
            && rd < SP
            && rn < SP
            && rm < SP
        {
            let (a, b) = (self.r[rn], self.r[rm]);
            self.r[rd] = if b == 0 {
                0
            } else if h & 0x20 != 0 {
                a / b
            } else {
                (a as i32).wrapping_div(b as i32) as u32
            };
            // PM0056: латентность деления зависит от операндов. Оценка по верхней границе Cortex-M3.
            return Ok(Some(12));
        }
        if h & 0xfff0 == 0xfb00 && l & 0x00f0 == 0 && rd < SP && rn < SP && rm < SP {
            let ra = usize::from(l >> 12);
            if ra == SP {
                return Ok(None);
            }
            self.r[rd] = self.r[rn]
                .wrapping_mul(self.r[rm])
                .wrapping_add(if ra == 15 { 0 } else { self.r[ra] });
            return Ok(Some(2));
        }
        let (size, load, signed) = match h & 0xff70 {
            0xf840 => (4, false, false),
            0xf850 => (4, true, false),
            0xf820 => (2, false, false),
            0xf830 => (2, true, false),
            0xf800 => (1, false, false),
            0xf810 => (1, true, false),
            0xf910 => (1, true, true),
            0xf930 => (2, true, true),
            _ => return Ok(None),
        };
        let rt = usize::from(l >> 12);
        if rt == 15 || rt == SP && size != 4 || rn == 15 && !load {
            return Ok(None);
        }
        let base = if rn == 15 {
            pc.wrapping_add(4) & !3
        } else {
            self.r[rn]
        };
        let (addr, writeback) = if rn == 15 {
            let n = u32::from(l & 0xfff);
            (
                if h & 128 != 0 {
                    base.wrapping_add(n)
                } else {
                    base.wrapping_sub(n)
                },
                None,
            )
        } else if h & 128 != 0 {
            (base.wrapping_add(u32::from(l & 0xfff)), None)
        } else if l & 0xfc0 == 0 && rm < SP {
            (base.wrapping_add(self.r[rm] << (l >> 4 & 3)), None)
        } else if l & 0x800 != 0 {
            let index = l & 0x400 != 0;
            let wb = l & 0x100 != 0;
            if !wb || rn == rt || !index && l & 0x200 != 0 && rn == SP {
                return Ok(None);
            }
            let n = u32::from(l & 255);
            let updated = if l & 0x200 != 0 {
                base.wrapping_add(n)
            } else {
                base.wrapping_sub(n)
            };
            (if index { updated } else { base }, Some(updated))
        } else {
            return Ok(None);
        };
        if load {
            let raw = bus.read(pc, addr, size)?;
            let value = if signed && size == 1 {
                raw as i8 as i32 as u32
            } else if signed {
                raw as i16 as i32 as u32
            } else {
                raw
            };
            self.r[rt] = if rt == SP { value & !3 } else { value };
        } else {
            bus.write(pc, addr, size, self.r[rt])?;
        }
        if let Some(updated) = writeback {
            self.r[rn] = updated;
        }
        Ok(Some(2))
    }
}

fn sign(value: u32, bits: u32) -> u32 {
    (((value << (32 - bits)) as i32) >> (32 - bits)) as u32
}
