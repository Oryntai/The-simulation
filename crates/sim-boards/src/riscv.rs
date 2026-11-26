//! RV32IMC: Unprivileged ISA §§2, 13, 16, 27 и Privileged ISA §3.
//! Декодирование разделено с sim-mcu, карта памяти и прерывания принадлежат ESP32-C3.

use crate::{BoardFault, memory::Bus, peripheral::esp_pms::PmsOp};
use sim_mcu::cpu::{Op, cause, decode};

#[path = "peripheral/esp_irq.rs"]
mod esp_irq;
use esp_irq::EspIrq;

// TRM v1.4, регистры 1.5/1.7. В MPP записывается младший бит, старший его повторяет.
const MIE: u32 = 1 << 3;
const MPIE: u32 = 1 << 7;
const MPP_LOW: u32 = 1 << 11;
const MPP: u32 = 3 << 11;
const TW: u32 = 1 << 21;
const VECTOR_MODE: u32 = 1;
const VECTOR_BASE: u32 = !0xff;
// ESP-IDF components/riscv/include/riscv/encoding.h, USTATUS_{UIE,UPIE}.
const USER_STATUS_BITS: u32 = (1 << 0) | (1 << 4);
// TRM §3.3.2 и esp_rom.rs: холодный старт raw flash начинается в ROM0.
const RESET_VECTOR: u32 = 0x4000_0000;
// RISC-V Privileged ISA, таблица синхронных исключений mcause.
const ECALL_U: u32 = 8;

#[derive(Clone, Default)]
pub(crate) struct Riscv {
    r: [u32; 32],
    pub pc: u32,
    status: u32,
    user_status: u32,
    user: bool,
    vector: u32,
    epc: u32,
    cause: u32,
    trap_value: u32,
    scratch: u32,
    instret: u64,
    cycle_offset: u64,
    perf: Performance,
    pmp: Pmp,
    irq: EspIrq,
    sleeping: bool,
    wfi_cycles: u8,
    rom_profile: bool,
    rom_handoff_done: bool,
    rom_callback_returns: Vec<u32>,
}

impl Riscv {
    pub fn boot(bus: &Bus, entry: u32) -> Result<Self, BoardFault> {
        if entry & 1 != 0 {
            return Err(BoardFault::BadImage(
                "не выровнена точка входа RISC-V".into(),
            ));
        }
        bus.fetch(entry, 2)?;
        let rom_profile = entry == RESET_VECTOR;
        Ok(Self {
            pc: entry,
            vector: if rom_profile { VECTOR_MODE } else { 0 },
            rom_profile,
            cycle_offset: 0u64.wrapping_sub(bus.hw.cycles),
            ..Self::default()
        })
    }

    pub(crate) fn finish_rom_handoff(&mut self) {
        self.rom_handoff_done = true;
    }

    pub(crate) fn clock_stalled(&self, wait_gate_cycles: Option<u8>) -> bool {
        self.sleeping && wait_gate_cycles.is_some_and(|delay| self.wfi_cycles > delay)
    }

    fn trap(&mut self, bus: &Bus, code: u32, tval: u32) -> Result<(), BoardFault> {
        let base = self.vector & !3;
        if base == 0 {
            return Err(BoardFault::InvalidState {
                pc: self.pc,
                target: 0,
                why: "не задан mtvec",
            });
        }
        let target = base.wrapping_add(
            if code & cause::INTERRUPT != 0 && self.vector & 3 == VECTOR_MODE {
                size_of::<u32>() as u32 * (code & !cause::INTERRUPT)
            } else {
                0
            },
        );
        if let Err(error) = self
            .pmp
            .check(false, self.pc, target, 2, Pmp::X)
            .and_then(|()| bus.fetch(target, 2).map(|_| ()))
        {
            return Err(BoardFault::Lockup {
                pc: self.pc,
                why: format!("недоступен вектор ловушки {target:#010x}: {error}"),
            });
        }
        // RISC-V Privileged ISA: при входе сохраняются адрес, режим и прежний MIE.
        self.epc = self.pc;
        self.cause = code;
        self.trap_value = tval;
        self.status = (self.status & TW)
            | ((self.status & MIE) << (MPIE.trailing_zeros() - MIE.trailing_zeros()))
            | if self.user { 0 } else { MPP };
        self.user = false;
        self.pc = target;
        self.sleeping = false;
        Ok(())
    }

    pub fn step(&mut self, bus: &mut Bus) -> Result<u64, BoardFault> {
        self.sample_irq(bus, self.pc, None)?;
        if let Some(line) = self.irq.next() {
            self.sleeping = false;
            // TRM §1.5: _CPU_INT_ENABLE маскирует линию локально. В U-mode
            // прерывание Machine-mode принимается независимо от MIE.
            if self.user || self.status & MIE != 0 {
                self.trap(bus, cause::INTERRUPT | line, 0)?;
                self.irq.claim(line);
                self.perf.tick();
                return Ok(1);
            }
        }
        if self.sleeping {
            self.wfi_cycles = self.wfi_cycles.saturating_add(1);
            return Ok(1);
        }

        let pc = self.pc;
        // PMP проверяет всю инструкцию, в том числе слово с IALIGN=16.
        let (raw, fault_addr) = match self.fetch(bus) {
            Ok(raw) => (raw, None),
            Err(BoardFault::BusError { addr, .. }) if self.vector & !3 != 0 => (0, Some(addr)),
            Err(error) => return Err(error),
        };
        if let Some(addr) = fault_addr {
            self.trap(bus, cause::INSTR_ACCESS_FAULT, addr)?;
        } else {
            match self.execute(bus, raw) {
                Ok(true) => self.instret = self.instret.wrapping_add(1),
                Ok(false) => {}
                Err(error) => {
                    self.pc = pc;
                    // Модельные ошибки (неизвестный CSR, MMIO, отсутствующие данные)
                    // остаются видны хосту даже при установленном обработчике.
                    let (code, addr) = match &error {
                        BoardFault::BusError { addr, write, .. } => (
                            if *write {
                                cause::STORE_ACCESS_FAULT
                            } else {
                                cause::LOAD_ACCESS_FAULT
                            },
                            *addr,
                        ),
                        BoardFault::WriteToReadOnly { addr, .. } => {
                            (cause::STORE_ACCESS_FAULT, *addr)
                        }
                        BoardFault::Unaligned { addr, .. } => (
                            if matches!(decode(raw, pc).op, Op::Sb | Op::Sh | Op::Sw) {
                                cause::STORE_MISALIGNED
                            } else {
                                cause::LOAD_MISALIGNED
                            },
                            *addr,
                        ),
                        _ => return Err(error),
                    };
                    if self.vector & !3 == 0 {
                        return Err(error);
                    }
                    self.trap(bus, code, addr)?;
                }
            }
        }
        if let Some(boot) = &mut bus.hw.esp_boot {
            // RISC-V psABI: x2 — SP. Выборка модели после действия инструкции.
            boot.observe_cpu(pc, self.r[2]);
        }
        self.perf.tick();
        // Оценка времени: один такт на инструкцию, задержки шины не моделируются.
        Ok(1)
    }

    fn fetch(&self, bus: &Bus) -> Result<u32, BoardFault> {
        let pc = self.pc;
        self.pmp.check(self.user, pc, pc, 2, Pmp::X)?;
        self.check_pms(bus, pc, 2, PmsOp::Fetch)?;
        let low = bus.fetch(pc, 2)?;
        if low & 3 != 3 {
            return Ok(low);
        }
        self.pmp.check(self.user, pc, pc, 4, Pmp::X)?;
        self.check_pms(bus, pc, 4, PmsOp::Fetch)?;
        Ok(low | bus.fetch(pc.wrapping_add(2), 2)? << 16)
    }

    fn check_pms(&self, bus: &Bus, addr: u32, size: u8, op: PmsOp) -> Result<(), BoardFault> {
        if let Some(boot) = &bus.hw.esp_boot {
            boot.pms.check(self.user, self.pc, addr, size, op)?;
        }
        Ok(())
    }

    fn sample_irq(
        &mut self,
        bus: &mut Bus,
        pc: u32,
        status: Option<u32>,
    ) -> Result<(), BoardFault> {
        let boot = bus.hw.esp_boot.as_deref();
        let systimer = boot.and_then(|boot| boot.systimer_interrupt_status());
        let assist = boot.map(|boot| boot.assist_interrupt());
        let memory = boot.map(|boot| (boot.flash.interrupt_levels(), boot.pms.interrupts()));
        self.irq.sample_with_boot_sources(
            pc,
            status,
            bus.hw.rtc_interrupt_pending(),
            assist,
            memory,
            |addr| {
                if addr == esp_irq::SYSTIMER_STATUS
                    && let Some(levels) = systimer
                {
                    return Ok(levels);
                }
                bus.read(pc, addr, size_of::<u32>() as u8)
            },
        )
    }

    fn execute(&mut self, bus: &mut Bus, raw: u32) -> Result<bool, BoardFault> {
        use Op::*;
        let pc = self.pc;
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
        self.pc = next;
        let value = match d.op {
            Li => i,
            Jal | Jalr => {
                let target = if d.op == Jal {
                    i
                } else {
                    a.wrapping_add(i) & !1
                };
                if self.rom_handoff_done {
                    use crate::esp_rom;
                    match (esp_rom::contains(pc), esp_rom::contains(target)) {
                        (true, false) if matches!(d.rd, 1 | 5) => {
                            // RISC-V ISA JAL/JALR: x1 и x5 — регистры связи.
                            self.rom_callback_returns.push(next);
                        }
                        (false, true) if self.rom_callback_returns.last() == Some(&target) => {
                            self.rom_callback_returns.pop();
                        }
                        (false, true) => esp_rom::check_guest_entry(target)?,
                        _ => {}
                    }
                }
                self.pc = target;
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
                return Ok(true);
            }
            Lb | Lh | Lw | Lbu | Lhu => {
                let size = match d.op {
                    Lb | Lbu => 1,
                    Lh | Lhu => 2,
                    _ => 4,
                };
                let addr = a.wrapping_add(i);
                self.pmp.check(self.user, pc, addr, size, Pmp::R)?;
                self.check_pms(bus, addr, size, PmsOp::Read)?;
                let v = if EspIrq::handles(addr) {
                    self.sample_irq(bus, pc, Some(addr))?;
                    self.irq.access(pc, addr, size, None)?
                } else {
                    bus.read(pc, addr, size)?
                };
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
                let addr = a.wrapping_add(i);
                self.pmp.check(self.user, pc, addr, size, Pmp::W)?;
                self.check_pms(bus, addr, size, PmsOp::Write)?;
                if EspIrq::handles(addr) {
                    self.irq.access(pc, addr, size, Some(b))?;
                } else {
                    bus.write(pc, addr, size, b)?;
                }
                return Ok(true);
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
            Fence | FenceI => return Ok(true),
            Wfi => {
                if self.user && self.status & TW != 0 {
                    self.pc = pc;
                    self.trap(bus, cause::ILLEGAL_INSTRUCTION, raw)?;
                    return Ok(false);
                }
                self.sleeping = true;
                self.wfi_cycles = 0;
                return Ok(true);
            }
            // Прежний стенд завершал программы EBREAK. В холодном ROM-профиле
            // настроенный обработчик получает архитектурную ловушку.
            Ebreak if self.rom_profile && self.vector & !3 != 0 => {
                self.pc = pc;
                self.trap(bus, cause::BREAKPOINT, pc)?;
                return Ok(false);
            }
            Ebreak => return Err(BoardFault::Breakpoint { pc }),
            Ecall => {
                self.pc = pc;
                self.trap(bus, if self.user { ECALL_U } else { cause::ECALL_M }, 0)?;
                return Ok(false);
            }
            Mret => {
                if self.user {
                    self.pc = pc;
                    self.trap(bus, cause::ILLEGAL_INSTRUCTION, raw)?;
                    return Ok(false);
                }
                // Privileged ISA «Trap-Return Instructions»: MPP очищается до U,
                // MPIE становится единицей, а прежний MPIE возвращается в MIE.
                self.user = self.status & MPP == 0;
                self.status = (self.status & TW)
                    | MPIE
                    | ((self.status & MPIE) >> (MPIE.trailing_zeros() - MIE.trailing_zeros()));
                self.pc = self.epc;
                return Ok(true);
            }
            Csrrw | Csrrs | Csrrc | Csrrwi | Csrrsi | Csrrci => {
                // Zicsr: для RS/RC запись подавляется по номеру x0 или uimm=0,
                // значение регистрового операнда на принятие решения не влияет.
                let writes = matches!(d.op, Csrrw | Csrrwi) || d.rs1 != 0;
                let old = self.read_csr(pc, i, writes, bus.hw.cycles)?;
                // Privileged ISA, таблица 2.1: режим в [9:8], read-only в [11:10].
                if self.user && (i >> 8) & 3 != 0 || writes && i >> 10 == 3 {
                    self.pc = pc;
                    self.trap(bus, cause::ILLEGAL_INSTRUCTION, raw)?;
                    return Ok(false);
                }
                if writes {
                    let src = if matches!(d.op, Csrrwi | Csrrsi | Csrrci) {
                        u32::from(d.rs1)
                    } else {
                        a
                    };
                    let new = match d.op {
                        Csrrw | Csrrwi => src,
                        Csrrs | Csrrsi => old | src,
                        _ => old & !src,
                    };
                    self.write_csr(pc, i, new, bus.hw.cycles)?;
                }
                old
            }
            Illegal => return Err(unknown()),
        };
        if d.rd != 0 {
            self.r[d.rd as usize] = value;
        }
        Ok(true)
    }

    fn read_csr(&self, pc: u32, addr: u32, write: bool, now: u64) -> Result<u32, BoardFault> {
        let cycle = now.wrapping_add(self.cycle_offset);
        Ok(match addr {
            0x000 => self.user_status, // ESP-IDF riscv/encoding.h, CSR_USTATUS.
            // TRM §§1.4.1–1.4.2, регистры 1.1–1.11. В misa бит U описывает U-mode.
            0x300 => self.status,
            0x301 => sim_mcu::cpu::MISA | (1 << 20),
            0x305 => self.vector,
            0x340 => self.scratch,
            0x341 => self.epc,
            0x342 => self.cause,
            0x343 => self.trap_value,
            0xf11 => 0x612,
            0xf12 => 0x8000_0001,
            0xf13 => 1,
            0xf14 => 0,
            // TRM §1.8.4: четыре упакованных pmpcfg и 16 адресов pmpaddr.
            0x3a0..=0x3a3 => self.pmp.config[(addr - 0x3a0) as usize],
            0x3b0..=0x3bf => self.pmp.addr[(addr - 0x3b0) as usize],
            // TRM, рег. 1.12–1.14 и ESP-IDF riscv/rv_utils.h.
            // Связь представлений следует из штатного запуска: ROM включает
            // отсчёт через 0x800/0x801 в M-mode и читает 0x802 (0x400462cc);
            // bootloader_support/src/bootloader_random.c уже ждёт прироста
            // через M-PCCR без повторной конфигурации PCER/PCMR.
            0x7e0..=0x7e2 => self.perf.read(addr - 0x7e0),
            0x800..=0x802 => self.perf.read(addr - 0x800),
            // Privileged ISA, Zicntr: сохраняются алиасы счётчиков прежних ELF.
            0xb00 | 0xc00 => cycle as u32,
            0xb80 | 0xc80 => (cycle >> 32) as u32,
            0xb02 | 0xc02 => self.instret as u32,
            0xb82 | 0xc82 => (self.instret >> 32) as u32,
            _ => return Err(unsupported_csr(pc, addr, write)),
        })
    }

    fn write_csr(&mut self, pc: u32, addr: u32, value: u32, now: u64) -> Result<(), BoardFault> {
        match addr {
            0x000 => self.user_status = value & USER_STATUS_BITS,
            0x300 => {
                self.status =
                    (value & (TW | MIE | MPIE)) | if value & MPP_LOW != 0 { MPP } else { 0 };
            }
            0x301 => {} // TRM §1.4.1: misa — WARL с фиксированными полями.
            0x305 if !self.rom_profile && value & 3 == 0 => self.vector = value,
            0x305 => self.vector = (value & VECTOR_BASE) | VECTOR_MODE,
            0x340 => self.scratch = value,
            0x341 => self.epc = value & !1, // Privileged ISA, IALIGN=16 при расширении C.
            0x342 => self.cause = value, // TRM регистр 1.10: INTERRUPT[31], EXCEPTION_CODE[30:0].
            0x343 => self.trap_value = value,
            0x3a0..=0x3a3 => self.pmp.write_config((addr - 0x3a0) as usize, value),
            0x3b0..=0x3bf => self.pmp.write_addr((addr - 0x3b0) as usize, value),
            0x7e0..=0x7e2 => self.perf.write(pc, addr, addr - 0x7e0, value)?,
            0x800..=0x802 => self.perf.write(pc, addr, addr - 0x800, value)?,
            0xb00 | 0xb80 => {
                let cycle = now.wrapping_add(self.cycle_offset);
                self.cycle_offset = replace_half(cycle, addr == 0xb80, value).wrapping_sub(now);
            }
            0xb02 | 0xb82 => {
                // Privileged ISA: собственная запись minstret заменяет инкремент инструкции.
                self.instret = replace_half(self.instret, addr == 0xb82, value).wrapping_sub(1);
            }
            _ => return Err(unsupported_csr(pc, addr, true)),
        }
        Ok(())
    }
}

fn unsupported_csr(pc: u32, addr: u32, write: bool) -> BoardFault {
    BoardFault::UnsupportedRegister {
        pc,
        addr,
        write,
        size: size_of::<u32>() as u8,
        block: "CSR RV32",
    }
}

fn replace_half(old: u64, high: bool, value: u32) -> u64 {
    let shift = if high { u32::BITS } else { 0 };
    (old & !(u64::from(u32::MAX) << shift)) | (u64::from(value) << shift)
}

#[derive(Clone)]
struct Performance {
    select: u32,
    mode: u32,
    count: u32,
}

impl Default for Performance {
    fn default() -> Self {
        // TRM регистры 1.12–1.14: сброс PCER=0, PCMR=3, PCCR=0.
        Self {
            select: 0,
            mode: 3,
            count: 0,
        }
    }
}

impl Performance {
    // TRM регистры 1.12–1.13; ROM включает цикл и счётчик записью единицы.
    // COUNT_SAT находится в бите 1 и COUNT_EN в бите 0 (диаграмма рег. 1.13).
    const CYCLE: u32 = 1;
    const COUNT_EN: u32 = 1;
    const COUNT_SAT: u32 = 2;

    fn read(&self, field: u32) -> u32 {
        match field {
            0 => self.select,
            1 => self.mode,
            _ => self.count,
        }
    }

    fn write(&mut self, pc: u32, addr: u32, field: u32, value: u32) -> Result<(), BoardFault> {
        match field {
            0 if value & !Self::CYCLE == 0 => self.select = value,
            0 => return Err(unsupported_csr(pc, addr, true)),
            1 => self.mode = value & (Self::COUNT_EN | Self::COUNT_SAT),
            _ => self.count = value,
        }
        Ok(())
    }

    fn tick(&mut self) {
        if self.select & Self::CYCLE != 0 && self.mode & Self::COUNT_EN != 0 {
            self.count = if self.mode & Self::COUNT_SAT != 0 {
                self.count.saturating_add(1)
            } else {
                self.count.wrapping_add(1)
            };
        }
    }
}

// TRM §1.8 и Privileged ISA «Physical Memory Protection».
// Размеры, порядок адресов и биты следуют ESP-IDF riscv/csr.h (PMP_*).
#[derive(Clone, Default)]
struct Pmp {
    config: [u32; 4],
    addr: [u32; 16],
    locked: u16,
}

impl Pmp {
    const R: u8 = 1;
    const W: u8 = 1 << 1;
    const X: u8 = 1 << 2;
    const MODE: u8 = 3 << 3;
    const TOR: u8 = 1 << 3;
    const NA4: u8 = 2 << 3;
    const NAPOT: u8 = 3 << 3;
    const L: u8 = 1 << 7;
    const ADDR_MASK: u32 = u32::MAX >> 2; // Физический адрес ESP32-C3 32-битный.
    const MAX_NAPOT: u64 = 1 << 30; // TRM §1.8.2.

    fn config_byte(&self, n: usize) -> u8 {
        self.config[n / 4].to_le_bytes()[n % 4]
    }

    fn write_config(&mut self, index: usize, value: u32) {
        let mut bytes = self.config[index].to_le_bytes();
        for (offset, (old, new)) in bytes.iter_mut().zip(value.to_le_bytes()).enumerate() {
            if *old & Self::L != 0 {
                continue;
            }
            let mut legal = new & (Self::L | Self::MODE | Self::R | Self::W | Self::X);
            if legal & (Self::R | Self::W) == Self::W {
                legal &= !Self::W; // Зарезервированное сочетание R=0,W=1.
            }
            *old = legal;
            if legal & Self::L != 0 {
                self.locked |= 1 << (index * 4 + offset);
            }
        }
        self.config[index] = u32::from_le_bytes(bytes);
    }

    fn write_addr(&mut self, index: usize, value: u32) {
        if self.locked & (1 << index) != 0
            || (index + 1 < self.addr.len()
                && self.config_byte(index + 1) & (Self::L | Self::MODE) == Self::L | Self::TOR)
        {
            return;
        }
        self.addr[index] = value & Self::ADDR_MASK;
    }

    fn bounds(&self, n: usize, mode: u8, pc: u32) -> Result<Option<(u64, u64)>, BoardFault> {
        let word = u64::from(self.addr[n]);
        Ok(match mode {
            Self::TOR => {
                let start = if n == 0 {
                    0
                } else {
                    u64::from(self.addr[n - 1]) << 2
                };
                Some((start, word << 2))
            }
            Self::NA4 => Some((word << 2, (word + 1) << 2)),
            Self::NAPOT => {
                let ones = word.trailing_ones();
                let size = 1u64 << (ones + 3);
                if size > Self::MAX_NAPOT {
                    return Err(BoardFault::Unsupported(format!(
                        "PMP NAPOT больше 1 ГиБ, pmpaddr{n}, PC {pc:#010x}"
                    )));
                }
                let base = (word << 2) & !(size - 1);
                Some((base, base + size))
            }
            _ => None,
        })
    }

    fn check(
        &self,
        user: bool,
        pc: u32,
        addr: u32,
        size: u8,
        permission: u8,
    ) -> Result<(), BoardFault> {
        let mut entries = if user { u16::MAX } else { self.locked };
        let start = u64::from(addr);
        let end = start + u64::from(size);
        let mut matched = false;
        let mut allowed = false;
        while entries != 0 {
            let n = entries.trailing_zeros() as usize;
            entries &= entries - 1;
            let config = self.config_byte(n);
            if let Some((lo, hi)) = self.bounds(n, config & Self::MODE, pc)?
                && lo < hi
                && lo < end
                && start < hi
            {
                // TRM §1.8.2: приоритет перекрытий отличается от стандартного PMP.
                // Штатный ESP-IDF задаёт непересекающиеся границы. Остальные карты
                // требуют проверки на кремнии и здесь останавливают прогон.
                if matched {
                    return Err(BoardFault::Unsupported(format!(
                        "пересечение областей PMP для {addr:#010x}, PC {pc:#010x}"
                    )));
                }
                matched = true;
                allowed = lo <= start && end <= hi && config & permission != 0;
            }
        }
        if allowed || (!user && !matched) {
            Ok(())
        } else {
            Err(BoardFault::BusError {
                pc,
                addr,
                write: permission == Self::W,
            })
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        BoardId,
        esp_memory::EspMemory,
        image::{Image, Segment},
    };

    #[test]
    fn wfi_останавливает_pccr_и_instret_но_не_время_шины() {
        // TRM §3.3.2, начало SRAM1. RV32I: четыре CSR-инструкции и WFI.
        const ENTRY: u32 = 0x4038_0000;
        let image = Image {
            entry: Some(ENTRY),
            segments: vec![Segment {
                addr: ENTRY,
                data: [
                    0x7e0_0d073u32, // csrwi mpcer,1; TRM рег. 1.12.
                    0x7e1_0d073,    // csrwi mpcmr,1; TRM рег. 1.13.
                    0x7e2_05073,    // csrwi mpccr,0; TRM рег. 1.14.
                    0xb02_05073,    // csrwi minstret,0; Privileged ISA.
                    0x1050_0073,    // wfi.
                ]
                .into_iter()
                .flat_map(u32::to_le_bytes)
                .collect(),
            }],
        };
        let mut bus = Bus::new(BoardId::Esp32C3, &image).unwrap();
        let mut cpu = Riscv::boot(&bus, ENTRY).unwrap();
        for _ in 0..5 {
            let elapsed = cpu.step(&mut bus).unwrap();
            bus.hw.tick(elapsed);
        }
        assert!(cpu.sleeping);
        assert_eq!(cpu.instret, 1); // После записи считается только WFI.
        assert_eq!(cpu.perf.count, 3); // Запись PCCR, запись minstret и WFI.
        let after_wfi = cpu.pc;
        // Два дополнительных такта: оба должны уйти в ожидание.
        for _ in 0..2 {
            let elapsed = cpu.step(&mut bus).unwrap();
            bus.hw.tick(elapsed);
        }
        assert_eq!(cpu.pc, after_wfi);
        assert_eq!(cpu.perf.count, 3);
        assert_eq!(cpu.instret, 1);
        assert_eq!(bus.hw.cycles, 7);
    }

    #[test]
    #[ignore = "нужны ROM ESP32-C3 и список PC от QEMU Espressif"]
    fn потактовый_префикс_rom_совпадает_с_qemu() {
        let read = |key| std::fs::read(std::env::var(key).expect(key)).expect(key);
        let rom = read("SIM_ESP_BOOT_ROM");
        let readback = read("SIM_ESP_BOOT_READBACK");
        let pcs = String::from_utf8(read("SIM_ESP_BOOT_QEMU_PC")).unwrap();
        let mut memory = EspMemory::default();
        memory.install_rom(&rom).unwrap();
        memory
            .install_efuse_readback(
                &readback
                    .chunks_exact(size_of::<u32>())
                    .map(|bytes| u32::from_le_bytes(bytes.try_into().unwrap()))
                    .collect::<Vec<_>>(),
            )
            .unwrap();
        memory.latch_straps(true, true, true);
        let mut bus = Bus::new(
            BoardId::Esp32C3,
            &Image {
                entry: None,
                segments: vec![],
            },
        )
        .unwrap();
        bus.esp_memory = Some(memory);
        let mut cpu = Riscv::boot(&bus, RESET_VECTOR).unwrap();
        // Измерение QEMU 9.2.2: PC с one-insn-per-tb, от вектора до RTC_CNTL_RESET_STATE_REG.
        const STOP_AT: usize = 5523;
        let mut trace = pcs.lines();
        for index in 0..=STOP_AT {
            let expected = u32::from_str_radix(trace.next().unwrap(), 16).unwrap();
            assert_eq!(cpu.pc, expected, "расхождение PC на инструкции {index}");
            let step = cpu.step(&mut bus);
            if index == STOP_AT {
                assert!(matches!(
                    step,
                    Err(BoardFault::UnsupportedRegister {
                        pc: 0x4004_8a94,
                        addr: 0x6000_8038, // ESP-IDF esp32c3/register/soc/rtc_cntl_reg.h.
                        write: false,
                        size: 4,
                        ..
                    })
                ));
            } else {
                bus.hw.tick(step.unwrap());
            }
        }
    }
}
