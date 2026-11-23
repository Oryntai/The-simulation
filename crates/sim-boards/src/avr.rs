//! AVR: Microchip DS40002198C, кодировки, SREG и число тактов для AVRe.
//! PC внутри ядра хранится в байтах, адреса переходов AVR — в словах.

use crate::{BoardFault, memory::Bus};

const C: u8 = 1;
const Z: u8 = 2;
const N: u8 = 4;
const V: u8 = 8;
const S: u8 = 16;
const H: u8 = 32;
const I: u8 = 128;

#[derive(Clone)]
pub(crate) struct Avr {
    r: [u8; 32],
    pub pc: u32,
    sp: u16,
    sreg: u8,
    irq_delay: bool,
}

impl Default for Avr {
    fn default() -> Self {
        // 7810D §§7, 8: RAMEND = 0x08ff.
        Self {
            r: [0; 32],
            pc: 0,
            sp: 0x8ff,
            sreg: 0,
            irq_delay: false,
        }
    }
}

impl Avr {
    fn read(&mut self, bus: &mut Bus, pc: u32, addr: u16) -> Result<u8, BoardFault> {
        Ok(match addr {
            0..=31 => self.r[usize::from(addr)],
            0x5d => self.sp as u8,
            0x5e => (self.sp >> 8) as u8,
            0x5f => self.sreg,
            _ => bus.read(pc, u32::from(addr), 1)? as u8,
        })
    }

    fn write(&mut self, bus: &mut Bus, pc: u32, addr: u16, v: u8) -> Result<(), BoardFault> {
        match addr {
            0..=31 => self.r[usize::from(addr)] = v,
            0x5d => self.sp = (self.sp & 0xff00) | u16::from(v),
            0x5e => self.sp = (self.sp & 0xff) | (u16::from(v) << 8),
            0x5f => self.sreg = v,
            _ => bus.write(pc, u32::from(addr), 1, u32::from(v))?,
        }
        Ok(())
    }

    fn push(&mut self, bus: &mut Bus, pc: u32, v: u8) -> Result<(), BoardFault> {
        if !(0x100..=0x8ff).contains(&self.sp) {
            return Err(BoardFault::InvalidState {
                pc,
                target: u32::from(self.sp),
                why: "стек AVR вне SRAM",
            });
        }
        self.write(bus, pc, self.sp, v)?;
        self.sp -= 1;
        Ok(())
    }

    fn pop(&mut self, bus: &mut Bus, pc: u32) -> Result<u8, BoardFault> {
        self.sp = self.sp.wrapping_add(1);
        if !(0x100..=0x8ff).contains(&self.sp) {
            return Err(BoardFault::InvalidState {
                pc,
                target: u32::from(self.sp),
                why: "стек AVR вне SRAM",
            });
        }
        self.read(bus, pc, self.sp)
    }

    fn target(
        &mut self,
        bus: &mut Bus,
        pc: u32,
        target: u32,
        call: bool,
    ) -> Result<(), BoardFault> {
        if call {
            let bytes = ((self.pc / 2) as u16).to_le_bytes();
            self.push(bus, pc, bytes[0])?;
            self.push(bus, pc, bytes[1])?;
        }
        self.pc = target;
        Ok(())
    }

    fn nzvs(&mut self, value: u8, overflow: bool, chain: bool) {
        let zero = value == 0 && (!chain || self.sreg & Z != 0);
        let negative = value & 128 != 0;
        self.sreg = (self.sreg & !(Z | N | V | S))
            | (u8::from(zero) * Z)
            | (u8::from(negative) * N)
            | (u8::from(overflow) * V)
            | (u8::from(negative ^ overflow) * S);
    }

    fn math(&mut self, a: u8, b: u8, sub: bool, carry: bool, chain: bool) -> u8 {
        let c = u16::from(carry);
        let (value, cy, half, signed) = if sub {
            (
                a.wrapping_sub(b).wrapping_sub(c as u8),
                u16::from(a) < u16::from(b) + c,
                u16::from(a & 15) < u16::from(b & 15) + c,
                i16::from(a as i8) - i16::from(b as i8) - c as i16,
            )
        } else {
            (
                a.wrapping_add(b).wrapping_add(c as u8),
                u16::from(a) + u16::from(b) + c > 255,
                u16::from(a & 15) + u16::from(b & 15) + c > 15,
                i16::from(a as i8) + i16::from(b as i8) + c as i16,
            )
        };
        self.sreg = (self.sreg & !(C | H)) | (u8::from(cy) * C) | (u8::from(half) * H);
        self.nzvs(value, signed != i16::from(value as i8), chain);
        value
    }

    fn skip(&mut self, bus: &Bus) -> Result<u64, BoardFault> {
        let next = bus.fetch(self.pc, 2)? as u16;
        let wide =
            matches!(next & 0xfe0f, 0x9000 | 0x9200) || matches!(next & 0xfe0e, 0x940c | 0x940e);
        self.pc += if wide { 4 } else { 2 };
        Ok(if wide { 3 } else { 2 })
    }

    pub fn step(&mut self, bus: &mut Bus) -> Result<u64, BoardFault> {
        if self.sreg & I != 0
            && !self.irq_delay
            && let Some(vector) = bus.hw.avr_irq()
        {
            self.sreg &= !I;
            self.target(bus, self.pc, vector, true)?;
            return Ok(4);
        }
        self.irq_delay = false;
        let pc = self.pc;
        let h = bus.fetch(pc, 2)? as u16;
        self.pc = pc.wrapping_add(2);
        let unknown = || BoardFault::UnknownInstruction {
            pc,
            raw: u32::from(h),
            isa: "AVR (AVRe)",
        };
        let d = usize::from(h >> 4 & 31);
        let r = usize::from((h >> 5 & 16) | (h & 15));
        let k = ((h >> 4 & 0xf0) | (h & 15)) as u8;
        let mut cycles = 1;
        match h {
            0 => {}
            0x0100..=0x01ff => {
                let d = usize::from(h >> 3 & 30);
                let r = usize::from(h & 15) * 2;
                let pair = [self.r[r], self.r[r + 1]];
                self.r[d..d + 2].copy_from_slice(&pair);
            }
            0x0400..=0x2fff => {
                let a = self.r[d];
                let b = self.r[r];
                let carry = self.sreg & C != 0;
                match h & 0xfc00 {
                    0x0400 => {
                        self.math(a, b, true, carry, true);
                    }
                    0x0800 => self.r[d] = self.math(a, b, true, carry, true),
                    0x0c00 => self.r[d] = self.math(a, b, false, false, false),
                    0x1000 => {
                        if a == b {
                            return self.skip(bus);
                        }
                    }
                    0x1400 => {
                        self.math(a, b, true, false, false);
                    }
                    0x1800 => self.r[d] = self.math(a, b, true, false, false),
                    0x1c00 => self.r[d] = self.math(a, b, false, carry, false),
                    0x2000 | 0x2400 | 0x2800 => {
                        let value = match h & 0xfc00 {
                            0x2000 => a & b,
                            0x2400 => a ^ b,
                            _ => a | b,
                        };
                        self.nzvs(value, false, false);
                        self.r[d] = value;
                    }
                    _ => self.r[d] = b,
                }
            }
            0x3000..=0x7fff | 0xe000..=0xefff => {
                let d = 16 + usize::from(h >> 4 & 15);
                let a = self.r[d];
                match h >> 12 {
                    3 => {
                        self.math(a, k, true, false, false);
                    }
                    4 => self.r[d] = self.math(a, k, true, self.sreg & C != 0, true),
                    5 => self.r[d] = self.math(a, k, true, false, false),
                    6 | 7 => {
                        let value = if h >> 12 == 6 { a | k } else { a & k };
                        self.nzvs(value, false, false);
                        self.r[d] = value;
                    }
                    _ => self.r[d] = k,
                }
            }
            _ if h & 0xd000 == 0x8000 => {
                let ptr = if h & 8 == 0 { 30 } else { 28 };
                let offset = (h & 7) | (h >> 7 & 0x18) | (h >> 8 & 0x20);
                let address =
                    u16::from_le_bytes([self.r[ptr], self.r[ptr + 1]]).wrapping_add(offset);
                if h & 0x200 == 0 {
                    self.r[d] = self.read(bus, pc, address)?;
                } else {
                    self.write(bus, pc, address, self.r[d])?;
                }
                cycles = 2;
            }
            _ if matches!(h & 0xfe0f, 0x9000 | 0x9200) => {
                let address = bus.fetch(self.pc, 2)? as u16;
                self.pc += 2;
                if h & 0x200 == 0 {
                    self.r[d] = self.read(bus, pc, address)?;
                } else {
                    self.write(bus, pc, address, self.r[d])?;
                }
                cycles = 2;
            }
            _ if h & 0xfc00 == 0x9000 && matches!(h & 15, 1 | 2 | 9 | 10 | 12..=14) => {
                let ptr = match h & 15 {
                    1 | 2 => 30,
                    9 | 10 => 28,
                    _ => 26,
                };
                let mode = h & 3;
                if mode != 0 && (d == ptr || d == ptr + 1) {
                    return Err(unknown());
                }
                let old = u16::from_le_bytes([self.r[ptr], self.r[ptr + 1]]);
                let addr = old.wrapping_sub(u16::from(mode == 2));
                if h & 0x200 == 0 {
                    self.r[d] = self.read(bus, pc, addr)?;
                } else {
                    self.write(bus, pc, addr, self.r[d])?;
                }
                self.r[ptr..ptr + 2]
                    .copy_from_slice(&addr.wrapping_add(u16::from(mode == 1)).to_le_bytes());
                cycles = 2;
            }
            _ if matches!(h & 0xfe0f, 0x9004 | 0x9005) || h == 0x95c8 => {
                let address = u16::from_le_bytes([self.r[30], self.r[31]]);
                if h & 1 != 0 && d >= 30 {
                    return Err(unknown());
                }
                self.r[if h == 0x95c8 { 0 } else { d }] = bus.fetch(u32::from(address), 1)? as u8;
                if h & 1 != 0 {
                    self.r[30..].copy_from_slice(&address.wrapping_add(1).to_le_bytes());
                }
                cycles = 3;
            }
            _ if h & 0xfe0f == 0x900f => {
                self.r[d] = self.pop(bus, pc)?;
                cycles = 2;
            }
            _ if h & 0xfe0f == 0x920f => {
                self.push(bus, pc, self.r[d])?;
                cycles = 2;
            }
            _ if h & 0xff0f == 0x9408 => {
                let bit = 1 << (h >> 4 & 7);
                if h & 128 == 0 {
                    self.sreg |= bit;
                    self.irq_delay = bit == I;
                } else {
                    self.sreg &= !bit;
                }
            }
            0x9409 | 0x9509 => {
                self.target(
                    bus,
                    pc,
                    2 * u32::from(u16::from_le_bytes([self.r[30], self.r[31]])),
                    h == 0x9509,
                )?;
                cycles = if h == 0x9509 { 3 } else { 2 };
            }
            0x9508 | 0x9518 => {
                let high = self.pop(bus, pc)?;
                let low = self.pop(bus, pc)?;
                self.pc = 2 * u32::from(u16::from_le_bytes([low, high]));
                if h == 0x9518 {
                    self.sreg |= I;
                    self.irq_delay = true;
                }
                cycles = 4;
            }
            0x9598 => return Err(BoardFault::Breakpoint { pc }),
            0x95a8 => bus.hw.feed_watchdog(),
            _ if matches!(h & 0xfe0e, 0x940c | 0x940e) => {
                let low = bus.fetch(self.pc, 2)?;
                self.pc += 2;
                let addr = low | (u32::from((h >> 3 & 0x3e) | (h & 1)) << 16);
                let call = h & 2 != 0;
                self.target(bus, pc, addr * 2, call)?;
                cycles = if call { 4 } else { 3 };
            }
            _ if h & 0xfe00 == 0x9400 => {
                let a = self.r[d];
                self.r[d] = match h & 15 {
                    0 => {
                        let v = !a;
                        self.sreg |= C;
                        self.nzvs(v, false, false);
                        v
                    }
                    1 => self.math(0, a, true, false, false),
                    2 => a.rotate_left(4),
                    3 | 10 => {
                        let inc = h & 15 == 3;
                        let v = if inc {
                            a.wrapping_add(1)
                        } else {
                            a.wrapping_sub(1)
                        };
                        self.nzvs(v, a == if inc { 127 } else { 128 }, false);
                        v
                    }
                    5..=7 => {
                        let top = if h & 15 == 5 {
                            a & 128
                        } else if h & 15 == 7 {
                            (self.sreg & C) << 7
                        } else {
                            0
                        };
                        let value = (a >> 1) | top;
                        self.sreg = (self.sreg & !C) | (a & 1);
                        self.nzvs(value, (value & 128 != 0) ^ (a & 1 != 0), false);
                        value
                    }
                    _ => return Err(unknown()),
                };
            }
            0x9600..=0x97ff => {
                let d = 24 + usize::from(h >> 3 & 6);
                let a = u16::from_le_bytes([self.r[d], self.r[d + 1]]);
                let b = ((h >> 2) & 0x30) | (h & 15);
                let sub = h & 0x100 != 0;
                let value = if sub {
                    a.wrapping_sub(b)
                } else {
                    a.wrapping_add(b)
                };
                let (an, vn) = (a >> 15 != 0, value >> 15 != 0);
                let (carry, overflow) = if sub {
                    (!an && vn, an && !vn)
                } else {
                    (an && !vn, !an && vn)
                };
                self.sreg = (self.sreg & !(C | Z | N | V | S))
                    | (u8::from(carry) * C)
                    | (u8::from(value == 0) * Z)
                    | (u8::from(vn) * N)
                    | (u8::from(overflow) * V)
                    | (u8::from(vn ^ overflow) * S);
                self.r[d..d + 2].copy_from_slice(&value.to_le_bytes());
                cycles = 2;
            }
            0x9800..=0x9bff => {
                let addr = 0x20 + (h >> 3 & 31);
                let bit = 1 << (h & 7);
                let old = self.read(bus, pc, addr)?;
                let set = h & 0x200 != 0;
                if h & 0x100 != 0 {
                    if (old & bit != 0) == set {
                        return self.skip(bus);
                    }
                } else {
                    self.write(bus, pc, addr, if set { old | bit } else { old & !bit })?;
                    cycles = 2;
                }
            }
            0x9c00..=0x9fff | 0x0200..=0x02ff => {
                let value = if h & 0xff00 == 0x0200 {
                    let a = self.r[16 + usize::from(h >> 4 & 15)] as i8;
                    let b = self.r[16 + usize::from(h & 15)] as i8;
                    (i16::from(a) * i16::from(b)) as u16
                } else {
                    u16::from(self.r[d]) * u16::from(self.r[r])
                };
                self.r[..2].copy_from_slice(&value.to_le_bytes());
                self.sreg =
                    (self.sreg & !(C | Z)) | (u8::from(value == 0) * Z) | ((value >> 15) as u8);
                cycles = 2;
            }
            0xb000..=0xbfff => {
                let addr = 0x20 + ((h >> 5 & 0x30) | (h & 15));
                if h & 0x800 == 0 {
                    self.r[d] = self.read(bus, pc, addr)?;
                } else {
                    self.write(bus, pc, addr, self.r[d])?;
                }
            }
            0xc000..=0xdfff => {
                let delta = (((h << 4) as i16 >> 4) as i32 * 2) as u32;
                let call = h & 0x1000 != 0;
                self.target(bus, pc, self.pc.wrapping_add(delta), call)?;
                cycles = if call { 3 } else { 2 };
            }
            0xf000..=0xf7ff => {
                if (self.sreg & (1 << (h & 7)) != 0) == (h & 0x400 == 0) {
                    let delta = (((h >> 3 & 127) as i8) << 1) >> 1;
                    self.pc = self.pc.wrapping_add((i32::from(delta) * 2) as u32);
                    cycles = 2;
                }
            }
            _ if matches!(h & 0xfe08, 0xf800 | 0xfa00 | 0xfc00 | 0xfe00) => {
                let mask = 1 << (h & 7);
                match h >> 9 & 3 {
                    0 => {
                        self.r[d] = (self.r[d] & !mask) | if self.sreg & 64 != 0 { mask } else { 0 }
                    }
                    1 => self.sreg = (self.sreg & !64) | (u8::from(self.r[d] & mask != 0) << 6),
                    op => {
                        if (self.r[d] & mask != 0) == (op == 3) {
                            return self.skip(bus);
                        }
                    }
                }
            }
            _ => return Err(unknown()),
        }
        Ok(cycles)
    }
}
