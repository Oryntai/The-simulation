//! Самопроверка ядра VMCU-C3.
//!
//! Печатает в UART строки `ST <ключ> <hex>` (результаты `selftest-core`, хост
//! сравнивает их с нативным расчётом тех же функций) и `SYS <ключ> <hex>`
//! (системное: CSR, ловушки, период таймера, приём UART — хост сверяет со
//! спецификацией). В конце — `ST end <число ключей>` и остановка с кодом 0.
//!
//! Протокол с хостом для проверки приёма: прошивка печатает `SYS rx.ready 1` и
//! ждёт 16 байт по UART (через внешнее прерывание), затем печатает их CRC-32.
#![no_std]
#![no_main]

use core::arch::asm;
use core::hint::black_box;

use selftest_core::{ImmOp, Isa, LoadOp, RrOp, StoreOp};
use vmcu_hal::interrupt::{self, Global};
use vmcu_hal::rt::{self, TrapFrame};
use vmcu_hal::{csr, csr_read, intc, sys, timer, uart};

vmcu_hal::entry!(main);

/// Настоящие инструкции через `asm!`.
struct AsmIsa;

macro_rules! rr {
    ($ins:literal, $a:expr, $b:expr) => {{
        let r: u32;
        // SAFETY: чистая арифметика над регистрами.
        unsafe {
            asm!(concat!($ins, " {r}, {a}, {b}"), a = in(reg) $a, b = in(reg) $b, r = out(reg) r,
                 options(nomem, nostack))
        };
        r
    }};
}

macro_rules! imm {
    ($ins:literal, $a:expr) => {{
        let r: u32;
        // SAFETY: чистая арифметика над регистрами.
        unsafe { asm!(concat!($ins), a = in(reg) $a, r = out(reg) r, options(nomem, nostack)) };
        r
    }};
}

macro_rules! br {
    ($ins:literal, $a:expr, $b:expr) => {{
        let r: u32;
        // SAFETY: переход внутри вставки, память не трогается.
        unsafe {
            asm!(concat!("li {r}, 1\n", $ins, " {a}, {b}, 2f\n", "li {r}, 0\n", "2:"),
                 a = in(reg) $a, b = in(reg) $b, r = out(reg) r, options(nomem, nostack))
        };
        r
    }};
}

impl Isa for AsmIsa {
    fn rr(&self, op: RrOp, a: u32, b: u32) -> u32 {
        match op {
            RrOp::Add => rr!("add", a, b),
            RrOp::Sub => rr!("sub", a, b),
            RrOp::Sll => rr!("sll", a, b),
            RrOp::Slt => rr!("slt", a, b),
            RrOp::Sltu => rr!("sltu", a, b),
            RrOp::Xor => rr!("xor", a, b),
            RrOp::Srl => rr!("srl", a, b),
            RrOp::Sra => rr!("sra", a, b),
            RrOp::Or => rr!("or", a, b),
            RrOp::And => rr!("and", a, b),
            RrOp::Mul => rr!("mul", a, b),
            RrOp::Mulh => rr!("mulh", a, b),
            RrOp::Mulhsu => rr!("mulhsu", a, b),
            RrOp::Mulhu => rr!("mulhu", a, b),
            RrOp::Div => rr!("div", a, b),
            RrOp::Divu => rr!("divu", a, b),
            RrOp::Rem => rr!("rem", a, b),
            RrOp::Remu => rr!("remu", a, b),
        }
    }

    fn imm(&self, op: ImmOp, a: u32) -> u32 {
        match op {
            ImmOp::AddiM2048 => imm!("addi {r}, {a}, -2048", a),
            ImmOp::Addi2047 => imm!("addi {r}, {a}, 2047", a),
            ImmOp::SltiM1 => imm!("slti {r}, {a}, -1", a),
            ImmOp::SltiuM1 => imm!("sltiu {r}, {a}, -1", a),
            ImmOp::Sltiu1 => imm!("sltiu {r}, {a}, 1", a),
            ImmOp::XoriM1 => imm!("xori {r}, {a}, -1", a),
            ImmOp::Ori555 => imm!("ori {r}, {a}, 0x555", a),
            ImmOp::AndiM16 => imm!("andi {r}, {a}, -16", a),
            ImmOp::Slli0 => imm!("slli {r}, {a}, 0", a),
            ImmOp::Slli1 => imm!("slli {r}, {a}, 1", a),
            ImmOp::Slli31 => imm!("slli {r}, {a}, 31", a),
            ImmOp::Srli1 => imm!("srli {r}, {a}, 1", a),
            ImmOp::Srli31 => imm!("srli {r}, {a}, 31", a),
            ImmOp::Srai1 => imm!("srai {r}, {a}, 1", a),
            ImmOp::Srai31 => imm!("srai {r}, {a}, 31", a),
        }
    }

    fn branches(&self, a: u32, b: u32) -> u32 {
        br!("beq", a, b)
            | br!("bne", a, b) << 1
            | br!("blt", a, b) << 2
            | br!("bge", a, b) << 3
            | br!("bltu", a, b) << 4
            | br!("bgeu", a, b) << 5
    }

    fn load(&self, op: LoadOp, buf: &[u8; 16], off: usize) -> u32 {
        let p = buf.as_ptr().wrapping_add(off);
        let r: u32;
        // SAFETY: p внутри buf и выровнен для размера (так перебирает run_isa).
        unsafe {
            match op {
                LoadOp::Lb => {
                    asm!("lb {r}, 0({p})", p = in(reg) p, r = out(reg) r, options(readonly, nostack))
                }
                LoadOp::Lbu => {
                    asm!("lbu {r}, 0({p})", p = in(reg) p, r = out(reg) r, options(readonly, nostack))
                }
                LoadOp::Lh => {
                    asm!("lh {r}, 0({p})", p = in(reg) p, r = out(reg) r, options(readonly, nostack))
                }
                LoadOp::Lhu => {
                    asm!("lhu {r}, 0({p})", p = in(reg) p, r = out(reg) r, options(readonly, nostack))
                }
                LoadOp::Lw => {
                    asm!("lw {r}, 0({p})", p = in(reg) p, r = out(reg) r, options(readonly, nostack))
                }
            }
        }
        r
    }

    fn store(&self, op: StoreOp, buf: &mut [u8; 16], off: usize, v: u32) {
        let p = buf.as_mut_ptr().wrapping_add(off);
        // SAFETY: p внутри buf и выровнен для размера.
        unsafe {
            match op {
                StoreOp::Sb => {
                    asm!("sb {v}, 0({p})", p = in(reg) p, v = in(reg) v, options(nostack))
                }
                StoreOp::Sh => {
                    asm!("sh {v}, 0({p})", p = in(reg) p, v = in(reg) v, options(nostack))
                }
                StoreOp::Sw => {
                    asm!("sw {v}, 0({p})", p = in(reg) p, v = in(reg) v, options(nostack))
                }
            }
        }
    }

    fn csr_sequence(&self, v: [u32; 3]) -> [u32; 8] {
        let mut o = [0u32; 8];
        // SAFETY: mscratch прошивка больше нигде не использует.
        unsafe {
            asm!("csrw mscratch, zero");
            asm!("csrrw {o}, mscratch, {v}", o = out(reg) o[0], v = in(reg) v[0]);
            asm!("csrrs {o}, mscratch, {v}", o = out(reg) o[1], v = in(reg) v[1]);
            asm!("csrrc {o}, mscratch, {v}", o = out(reg) o[2], v = in(reg) v[2]);
            asm!("csrrwi {o}, mscratch, 21", o = out(reg) o[3]);
            asm!("csrrsi {o}, mscratch, 10", o = out(reg) o[4]);
            asm!("csrrci {o}, mscratch, 5", o = out(reg) o[5]);
            asm!("csrrs {o}, mscratch, zero", o = out(reg) o[6]);
            asm!("csrr {o}, mscratch", o = out(reg) o[7]);
        }
        o
    }

    fn upper(&self) -> [u32; 2] {
        let lui: u32;
        let diff: u32;
        // SAFETY: только регистры.
        unsafe {
            asm!("lui {r}, 0xfffff", r = out(reg) lui, options(nomem, nostack));
            asm!("2:", "auipc {t}, 0", "la {u}, 2b", "sub {r}, {t}, {u}",
                 t = out(reg) _, u = out(reg) _, r = out(reg) diff, options(nomem, nostack));
        }
        [lui, diff]
    }
}

fn kv(tag: &[u8], key: &str, v: u32) {
    uart::write_bytes(tag);
    uart::write_byte(b' ');
    uart::write_bytes(key.as_bytes());
    uart::write_byte(b' ');
    uart::write_hex(v);
    uart::write_byte(b'\n');
}

fn sys_kv(key: &str, v: u32) {
    kv(b"SYS", key, v);
}

/// Состояние ловушек: ожидается ли исключение, что пришло последним.
struct TrapLog {
    expect: bool,
    count: u32,
    mcause: u32,
    mepc: u32,
    mtval: u32,
}

static TRAPS: Global<TrapLog> = Global::new(TrapLog {
    expect: false,
    count: 0,
    mcause: 0,
    mepc: 0,
    mtval: 0,
});

/// Номер «системного вызова» для проверки ecall.
const SYSCALL_DOUBLE: u32 = 0x5a;

fn exception(frame: &mut TrapFrame, mcause: u32, mepc: u32, mtval: u32) -> u32 {
    let expected = TRAPS.with(|t| {
        t.count += 1;
        t.mcause = mcause;
        t.mepc = mepc;
        t.mtval = mtval;
        let e = t.expect;
        t.expect = false;
        e
    });
    if mcause == 11 {
        if frame.a[7] == SYSCALL_DOUBLE {
            frame.a[0] = frame.a[0].wrapping_mul(2).wrapping_add(1);
        }
        return mepc + 4; // ecall не бывает сжатой
    }
    if expected {
        return mepc + rt::instruction_len(mepc);
    }
    rt::default_exception(frame, mcause, mepc, mtval)
}

fn ecall(num: u32, arg: u32) -> u32 {
    let r: u32;
    // SAFETY: ловушку обслуживает `exception`.
    unsafe { asm!("ecall", inlateout("a0") arg => r, in("a7") num) };
    r
}

fn expect_trap() {
    TRAPS.with(|t| t.expect = true);
}

fn last_trap() -> (u32, u32, u32) {
    TRAPS.with(|t| (t.mcause, t.mepc, t.mtval))
}

// ----- таймер -----

const TIMER_PERIOD: u64 = 1600; // тиков mtime (16 МГц) = 100 мкс
const STAMPS: usize = 21;

struct TimerLog {
    next: u64,
    n: usize,
    stamp: [u64; STAMPS],
    mcause: u32,
}

static TIMER: Global<TimerLog> = Global::new(TimerLog {
    next: 0,
    n: 0,
    stamp: [0; STAMPS],
    mcause: 0,
});

fn timer_isr() {
    let c = csr::mcycle();
    let mcause = csr_read!("mcause");
    TIMER.with(|t| {
        if t.n < STAMPS {
            t.stamp[t.n] = c;
            t.n += 1;
        }
        t.mcause = mcause;
        t.next += TIMER_PERIOD;
        timer::set_compare(if t.n < STAMPS { t.next } else { u64::MAX });
    });
}

// ----- приём UART -----

struct RxLog {
    buf: [u8; 16],
    n: usize,
    mcause: u32,
}

static RX: Global<RxLog> = Global::new(RxLog {
    buf: [0; 16],
    n: 0,
    mcause: 0,
});

fn external_isr() {
    let mcause = csr_read!("mcause");
    while let Some(b) = uart::read_byte() {
        RX.with(|r| {
            if r.n < r.buf.len() {
                r.buf[r.n] = b;
                r.n += 1;
            }
            r.mcause = mcause;
        });
    }
}

fn main() -> ! {
    vmcu_hal::println!("ST begin");
    let mut n = 0u32;
    {
        let mut emit = |k: &'static str, v: u32| {
            kv(b"ST", k, v);
            n += 1;
        };
        selftest_core::run_isa(&AsmIsa, &mut emit);
        selftest_core::run_algo(&mut emit);
    }

    // CSR-идентификация.
    sys_kv("misa", csr_read!("misa"));
    sys_kv("mhartid", csr_read!("mhartid"));
    sys_kv("mstatus.mpp", (csr_read!("mstatus") >> 11) & 3);
    sys_kv("sysctrl.id", sys::id());
    sys_kv("reset_reason", sys::reset_reason());

    // Счётчики: 100 итераций цикла.
    let c0 = csr::mcycle();
    let i0 = csr::minstret();
    let mut acc = 0u32;
    for k in 0..black_box(100u32) {
        acc = acc.wrapping_add(black_box(k));
    }
    black_box(acc);
    let c1 = csr::mcycle();
    let i1 = csr::minstret();
    sys_kv("mcycle.grows", (c1 > c0) as u32);
    sys_kv("minstret.grows", (i1 > i0 + 100) as u32);
    sys_kv(
        "cycles_per_instr_x100",
        ((c1 - c0) * 100 / (i1 - i0).max(1)) as u32,
    );

    // Ловушки.
    rt::set_exception_handler(exception);
    sys_kv("ecall.ret", ecall(SYSCALL_DOUBLE, 21));
    sys_kv("ecall.mcause", last_trap().0);

    // c.unimp (полуслово из нулей) — недопустимая инструкция длиной 2.
    expect_trap();
    // SAFETY: ловушка ожидается и пропускает инструкцию.
    unsafe { asm!(".2byte 0x0000") };
    let (c, _, v) = last_trap();
    sys_kv("trap.illegal16.mcause", c);
    sys_kv("trap.illegal16.mtval", v);

    // csrrw x0, cycle, x0: запись в CSR только для чтения — недопустимая.
    expect_trap();
    // SAFETY: см. выше.
    unsafe { asm!(".4byte 0xc0001073") };
    let (c, _, v) = last_trap();
    sys_kv("trap.illegal32.mcause", c);
    sys_kv("trap.illegal32.mtval", v);

    // Невыровненная загрузка слова.
    let buf = [0u32; 4];
    let addr = (buf.as_ptr() as u32) + 1;
    expect_trap();
    // SAFETY: доступ ловится исключением, результат не используется.
    unsafe { asm!("lw {r}, 0({p})", p = in(reg) addr, r = out(reg) _) };
    let (c, _, v) = last_trap();
    sys_kv("trap.lw_misaligned.mcause", c);
    sys_kv("trap.lw_misaligned.tval_ok", (v == addr) as u32);

    // Загрузка из несуществующей памяти.
    expect_trap();
    // SAFETY: см. выше.
    unsafe { asm!("lw {r}, 0({p})", p = in(reg) 0x1000_0000u32, r = out(reg) _) };
    let (c, _, v) = last_trap();
    sys_kv("trap.lw_unmapped.mcause", c);
    sys_kv("trap.lw_unmapped.mtval", v);

    // Запись во флеш.
    expect_trap();
    // SAFETY: см. выше.
    unsafe { asm!("sw zero, 0({p})", p = in(reg) 0x4200_0000u32) };
    let (c, _, v) = last_trap();
    sys_kv("trap.sw_flash.mcause", c);
    sys_kv("trap.sw_flash.mtval", v);
    sys_kv("trap.count", TRAPS.with(|t| t.count));

    // Периодическое прерывание таймера 10 кГц.
    rt::set_timer_handler(timer_isr);
    TIMER.with(|t| {
        t.next = timer::mtime() + TIMER_PERIOD;
        timer::set_compare(t.next);
    });
    timer::enable_irq();
    interrupt::enable();
    while TIMER.with(|t| t.n) < STAMPS {
        interrupt::wait();
    }
    let (mut pmin, mut pmax, mut psum) = (u64::MAX, 0u64, 0u64);
    TIMER.with(|t| {
        for k in 1..STAMPS {
            let p = t.stamp[k] - t.stamp[k - 1];
            pmin = pmin.min(p);
            pmax = pmax.max(p);
            psum += p;
        }
    });
    sys_kv("timer.period_cfg_cycles", (TIMER_PERIOD * 10) as u32);
    sys_kv("timer.period_min", pmin as u32);
    sys_kv("timer.period_max", pmax as u32);
    sys_kv("timer.period_sum20", psum as u32);
    sys_kv("timer.mcause", TIMER.with(|t| t.mcause));

    // Приём UART через внешнее прерывание.
    rt::set_external_handler(external_isr);
    intc::enable(intc::UART_RX);
    sys_kv("rx.ready", 1);
    while RX.with(|r| r.n) < 16 {
        interrupt::wait();
    }
    let (crc, mc) = RX.with(|r| (selftest_core::crc32(&r.buf), r.mcause));
    sys_kv("rx.crc", crc);
    sys_kv("rx.mcause", mc);

    vmcu_hal::println!("ST end {}", n);
    sys::halt(0)
}
