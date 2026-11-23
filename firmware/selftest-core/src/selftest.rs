// Тело `selftest-core` (включается через `include!` в прошивку и в хостовый sim-mcu).
// Только `core`, без выделения памяти. Входы алгоритмов пропущены через
// `black_box`, чтобы компилятор не посчитал результат заранее (иначе прошивка
// печатала бы константы и эмулятор ничего бы не исполнял).

use core::hint::black_box;

/// CRC-32 IEEE 802.3 (отражённый полином 0xEDB88320), табличный.
pub struct Crc32(u32);

const fn crc_table() -> [u32; 256] {
    let mut t = [0u32; 256];
    let mut i = 0;
    while i < 256 {
        let mut c = i as u32;
        let mut k = 0;
        while k < 8 {
            c = if c & 1 != 0 { (c >> 1) ^ 0xedb8_8320 } else { c >> 1 };
            k += 1;
        }
        t[i] = c;
        i += 1;
    }
    t
}

static CRC_TABLE: [u32; 256] = crc_table();

impl Crc32 {
    pub const fn new() -> Crc32 {
        Crc32(0xffff_ffff)
    }

    pub fn update(&mut self, data: &[u8]) {
        for &b in data {
            self.0 = (self.0 >> 8) ^ CRC_TABLE[((self.0 ^ b as u32) & 0xff) as usize];
        }
    }

    pub fn update_u32(&mut self, v: u32) {
        self.update(&v.to_le_bytes());
    }

    pub fn finish(&self) -> u32 {
        !self.0
    }
}

impl Default for Crc32 {
    fn default() -> Self {
        Crc32::new()
    }
}

/// CRC-32 буфера. Контрольное значение для "123456789" — 0xCBF43926.
pub fn crc32(data: &[u8]) -> u32 {
    let mut c = Crc32::new();
    c.update(data);
    c.finish()
}

/// FNV-1a, 32 бита.
pub fn fnv1a32(data: &[u8]) -> u32 {
    let mut h = 0x811c_9dc5u32;
    for &b in data {
        h ^= b as u32;
        h = h.wrapping_mul(0x0100_0193);
    }
    h
}

/// Числа Фибоначчи в u64 (на RV32 — сложение с переносом из двух слов).
pub fn fib_u64(n: u32) -> u64 {
    let (mut a, mut b) = (0u64, 1u64);
    for _ in 0..n {
        let t = a.wrapping_add(b);
        a = b;
        b = t;
    }
    a
}

/// Генератор xorshift32 (Marsaglia, 2003).
pub struct XorShift32(pub u32);

impl XorShift32 {
    pub fn next_u32(&mut self) -> u32 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.0 = x;
        x
    }
}

/// Пирамидальная сортировка (без выделения памяти).
pub fn heap_sort(a: &mut [u32]) {
    fn sift(a: &mut [u32], mut root: usize, end: usize) {
        loop {
            let mut child = 2 * root + 1;
            if child >= end {
                break;
            }
            if child + 1 < end && a[child] < a[child + 1] {
                child += 1;
            }
            if a[root] >= a[child] {
                break;
            }
            a.swap(root, child);
            root = child;
        }
    }
    let n = a.len();
    for i in (0..n / 2).rev() {
        sift(a, i, n);
    }
    for end in (1..n).rev() {
        a.swap(0, end);
        sift(a, 0, end);
    }
}

/// Сортировка вставками (знаковые числа).
pub fn insertion_sort(a: &mut [i32]) {
    for i in 1..a.len() {
        let v = a[i];
        let mut j = i;
        while j > 0 && a[j - 1] > v {
            a[j] = a[j - 1];
            j -= 1;
        }
        a[j] = v;
    }
}

/// Целый квадратный корень (метод Ньютона), floor(√n).
pub fn isqrt_u64(n: u64) -> u64 {
    if n < 2 {
        return n;
    }
    let mut x = n;
    let mut y = x.div_ceil(2);
    while y < x {
        x = y;
        y = (x + n / x) / 2;
    }
    x
}

/// Сумма 1/k² (k = 1..=n) в f32 — программная плавающая точка на RV32IMC.
pub fn f32_basel(n: u32) -> f32 {
    let mut s = 0.0f32;
    for k in 1..=n {
        let kf = k as f32;
        s += 1.0 / (kf * kf);
    }
    s
}

/// Ряд Лейбница 4·Σ(−1)^k/(2k+1) в f64.
pub fn f64_leibniz(n: u32) -> f64 {
    let mut s = 0.0f64;
    let mut sign = 1.0f64;
    for k in 0..n {
        s += sign / (2 * k + 1) as f64;
        sign = -sign;
    }
    4.0 * s
}

/// Смесь 64-битных умножений и делений (на RV32 — библиотечные __udivdi3 и т. п.).
pub fn u64_divmix(seed: u32, n: u32) -> u64 {
    let mut r = XorShift32(seed);
    let mut acc = 0x0123_4567_89ab_cdefu64;
    for _ in 0..n {
        let a = ((r.next_u32() as u64) << 32) | r.next_u32() as u64;
        let b = (r.next_u32() as u64) | 1;
        acc = acc.wrapping_mul(31).wrapping_add(a / b) ^ (a % b);
        let s = (a as i64) / ((b as i64) - 0x7fff_ffff);
        acc = acc.wrapping_add(s as u64);
    }
    acc
}

/// Степень целочисленной матрицы 3×3 (переполнение — по модулю 2³²).
pub fn matpow3(m: [[i32; 3]; 3], p: u32) -> [[i32; 3]; 3] {
    let mut r: [[i32; 3]; 3] = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
    for _ in 0..p {
        let mut t = [[0i32; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                let mut s = 0i32;
                for k in 0..3 {
                    s = s.wrapping_add(r[i][k].wrapping_mul(m[k][j]));
                }
                t[i][j] = s;
            }
        }
        r = t;
    }
    r
}

/// Эталонная семантика команд RV32IM по спецификации (Volume I, гл. 2 и 7),
/// записанная явными ветвлениями — независимо от кода эмулятора.
pub mod rv {
    pub fn add(a: u32, b: u32) -> u32 {
        a.wrapping_add(b)
    }
    pub fn sub(a: u32, b: u32) -> u32 {
        a.wrapping_sub(b)
    }
    /// Сдвиг на младшие 5 бит rs2.
    pub fn sll(a: u32, b: u32) -> u32 {
        a << (b % 32)
    }
    pub fn srl(a: u32, b: u32) -> u32 {
        a >> (b % 32)
    }
    pub fn sra(a: u32, b: u32) -> u32 {
        let s = b % 32;
        if a & 0x8000_0000 != 0 && s != 0 {
            (a >> s) | !(u32::MAX >> s)
        } else {
            a >> s
        }
    }
    pub fn slt(a: u32, b: u32) -> u32 {
        if (a as i32) < (b as i32) { 1 } else { 0 }
    }
    pub fn sltu(a: u32, b: u32) -> u32 {
        if a < b { 1 } else { 0 }
    }
    pub fn xor(a: u32, b: u32) -> u32 {
        a ^ b
    }
    pub fn or(a: u32, b: u32) -> u32 {
        a | b
    }
    pub fn and(a: u32, b: u32) -> u32 {
        a & b
    }
    pub fn mul(a: u32, b: u32) -> u32 {
        ((a as u64 * b as u64) & 0xffff_ffff) as u32
    }
    pub fn mulh(a: u32, b: u32) -> u32 {
        ((a as i32 as i64 * b as i32 as i64) >> 32) as u32
    }
    pub fn mulhsu(a: u32, b: u32) -> u32 {
        ((a as i32 as i64 * b as i64) >> 32) as u32
    }
    pub fn mulhu(a: u32, b: u32) -> u32 {
        ((a as u64 * b as u64) >> 32) as u32
    }
    /// Деление на 0 → все единицы; INT_MIN / −1 → INT_MIN (таблица 7.1).
    pub fn div(a: u32, b: u32) -> u32 {
        if b == 0 {
            0xffff_ffff
        } else if a == 0x8000_0000 && b == 0xffff_ffff {
            0x8000_0000
        } else {
            ((a as i32) / (b as i32)) as u32
        }
    }
    pub fn divu(a: u32, b: u32) -> u32 {
        match b {
            0 => 0xffff_ffff,
            _ => a / b,
        }
    }
    /// Остаток от деления на 0 → делимое; INT_MIN % −1 → 0.
    pub fn rem(a: u32, b: u32) -> u32 {
        if b == 0 {
            a
        } else if a == 0x8000_0000 && b == 0xffff_ffff {
            0
        } else {
            ((a as i32) % (b as i32)) as u32
        }
    }
    pub fn remu(a: u32, b: u32) -> u32 {
        match b {
            0 => a,
            _ => a % b,
        }
    }
}

/// Команды «регистр–регистр».
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RrOp {
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
}

impl RrOp {
    pub const ALL: [RrOp; 18] = [
        RrOp::Add,
        RrOp::Sub,
        RrOp::Sll,
        RrOp::Slt,
        RrOp::Sltu,
        RrOp::Xor,
        RrOp::Srl,
        RrOp::Sra,
        RrOp::Or,
        RrOp::And,
        RrOp::Mul,
        RrOp::Mulh,
        RrOp::Mulhsu,
        RrOp::Mulhu,
        RrOp::Div,
        RrOp::Divu,
        RrOp::Rem,
        RrOp::Remu,
    ];

    pub fn key(self) -> &'static str {
        match self {
            RrOp::Add => "rr.add",
            RrOp::Sub => "rr.sub",
            RrOp::Sll => "rr.sll",
            RrOp::Slt => "rr.slt",
            RrOp::Sltu => "rr.sltu",
            RrOp::Xor => "rr.xor",
            RrOp::Srl => "rr.srl",
            RrOp::Sra => "rr.sra",
            RrOp::Or => "rr.or",
            RrOp::And => "rr.and",
            RrOp::Mul => "rr.mul",
            RrOp::Mulh => "rr.mulh",
            RrOp::Mulhsu => "rr.mulhsu",
            RrOp::Mulhu => "rr.mulhu",
            RrOp::Div => "rr.div",
            RrOp::Divu => "rr.divu",
            RrOp::Rem => "rr.rem",
            RrOp::Remu => "rr.remu",
        }
    }
}

/// Команды с непосредственным операндом (операнд зашит в инструкцию).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ImmOp {
    AddiM2048,
    Addi2047,
    SltiM1,
    SltiuM1,
    Sltiu1,
    XoriM1,
    Ori555,
    AndiM16,
    Slli0,
    Slli1,
    Slli31,
    Srli1,
    Srli31,
    Srai1,
    Srai31,
}

impl ImmOp {
    pub const ALL: [ImmOp; 15] = [
        ImmOp::AddiM2048,
        ImmOp::Addi2047,
        ImmOp::SltiM1,
        ImmOp::SltiuM1,
        ImmOp::Sltiu1,
        ImmOp::XoriM1,
        ImmOp::Ori555,
        ImmOp::AndiM16,
        ImmOp::Slli0,
        ImmOp::Slli1,
        ImmOp::Slli31,
        ImmOp::Srli1,
        ImmOp::Srli31,
        ImmOp::Srai1,
        ImmOp::Srai31,
    ];

    pub fn key(self) -> &'static str {
        match self {
            ImmOp::AddiM2048 => "imm.addi_-2048",
            ImmOp::Addi2047 => "imm.addi_2047",
            ImmOp::SltiM1 => "imm.slti_-1",
            ImmOp::SltiuM1 => "imm.sltiu_-1",
            ImmOp::Sltiu1 => "imm.sltiu_1",
            ImmOp::XoriM1 => "imm.xori_-1",
            ImmOp::Ori555 => "imm.ori_0x555",
            ImmOp::AndiM16 => "imm.andi_-16",
            ImmOp::Slli0 => "imm.slli_0",
            ImmOp::Slli1 => "imm.slli_1",
            ImmOp::Slli31 => "imm.slli_31",
            ImmOp::Srli1 => "imm.srli_1",
            ImmOp::Srli31 => "imm.srli_31",
            ImmOp::Srai1 => "imm.srai_1",
            ImmOp::Srai31 => "imm.srai_31",
        }
    }

    /// Эталон: непосредственный операнд знаково расширяется до 32 бит.
    pub fn reference(self, a: u32) -> u32 {
        match self {
            ImmOp::AddiM2048 => rv::add(a, (-2048i32) as u32),
            ImmOp::Addi2047 => rv::add(a, 2047),
            ImmOp::SltiM1 => rv::slt(a, u32::MAX),
            ImmOp::SltiuM1 => rv::sltu(a, u32::MAX),
            ImmOp::Sltiu1 => rv::sltu(a, 1),
            ImmOp::XoriM1 => a ^ u32::MAX,
            ImmOp::Ori555 => a | 0x555,
            ImmOp::AndiM16 => a & 0xffff_fff0,
            ImmOp::Slli0 => rv::sll(a, 0),
            ImmOp::Slli1 => rv::sll(a, 1),
            ImmOp::Slli31 => rv::sll(a, 31),
            ImmOp::Srli1 => rv::srl(a, 1),
            ImmOp::Srli31 => rv::srl(a, 31),
            ImmOp::Srai1 => rv::sra(a, 1),
            ImmOp::Srai31 => rv::sra(a, 31),
        }
    }
}

/// Загрузки.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LoadOp {
    Lb,
    Lbu,
    Lh,
    Lhu,
    Lw,
}

/// Записи.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StoreOp {
    Sb,
    Sh,
    Sw,
}

/// Исполнитель команд: в прошивке — настоящие инструкции (`asm!`), на хосте —
/// эталонная семантика [`RefIsa`].
pub trait Isa {
    fn rr(&self, op: RrOp, a: u32, b: u32) -> u32;
    fn imm(&self, op: ImmOp, a: u32) -> u32;
    /// Биты исходов переходов: 0 beq, 1 bne, 2 blt, 3 bge, 4 bltu, 5 bgeu.
    fn branches(&self, a: u32, b: u32) -> u32;
    /// Загрузка из `buf` по смещению `off` (выровненному для размера).
    fn load(&self, op: LoadOp, buf: &[u8; 16], off: usize) -> u32;
    fn store(&self, op: StoreOp, buf: &mut [u8; 16], off: usize, v: u32);
    /// Последовательность csrrw/csrrs/csrrc/csrrwi/csrrsi/csrrci над `mscratch`
    /// (см. [`RefIsa::csr_sequence`]); возвращает прочитанные значения.
    fn csr_sequence(&self, v: [u32; 3]) -> [u32; 8];
    /// lui rd, 0xfffff (ожидается 0xFFFF_F000) и разность auipc − адрес метки (0).
    fn upper(&self) -> [u32; 2];
}

/// Эталонный исполнитель по спецификации.
pub struct RefIsa;

impl Isa for RefIsa {
    fn rr(&self, op: RrOp, a: u32, b: u32) -> u32 {
        match op {
            RrOp::Add => rv::add(a, b),
            RrOp::Sub => rv::sub(a, b),
            RrOp::Sll => rv::sll(a, b),
            RrOp::Slt => rv::slt(a, b),
            RrOp::Sltu => rv::sltu(a, b),
            RrOp::Xor => rv::xor(a, b),
            RrOp::Srl => rv::srl(a, b),
            RrOp::Sra => rv::sra(a, b),
            RrOp::Or => rv::or(a, b),
            RrOp::And => rv::and(a, b),
            RrOp::Mul => rv::mul(a, b),
            RrOp::Mulh => rv::mulh(a, b),
            RrOp::Mulhsu => rv::mulhsu(a, b),
            RrOp::Mulhu => rv::mulhu(a, b),
            RrOp::Div => rv::div(a, b),
            RrOp::Divu => rv::divu(a, b),
            RrOp::Rem => rv::rem(a, b),
            RrOp::Remu => rv::remu(a, b),
        }
    }

    fn imm(&self, op: ImmOp, a: u32) -> u32 {
        op.reference(a)
    }

    fn branches(&self, a: u32, b: u32) -> u32 {
        let (sa, sb) = (a as i32, b as i32);
        (a == b) as u32
            | ((a != b) as u32) << 1
            | ((sa < sb) as u32) << 2
            | ((sa >= sb) as u32) << 3
            | ((a < b) as u32) << 4
            | ((a >= b) as u32) << 5
    }

    fn load(&self, op: LoadOp, buf: &[u8; 16], off: usize) -> u32 {
        match op {
            LoadOp::Lb => buf[off] as i8 as i32 as u32,
            LoadOp::Lbu => buf[off] as u32,
            LoadOp::Lh => i16::from_le_bytes([buf[off], buf[off + 1]]) as i32 as u32,
            LoadOp::Lhu => u16::from_le_bytes([buf[off], buf[off + 1]]) as u32,
            LoadOp::Lw => u32::from_le_bytes([buf[off], buf[off + 1], buf[off + 2], buf[off + 3]]),
        }
    }

    fn store(&self, op: StoreOp, buf: &mut [u8; 16], off: usize, v: u32) {
        let b = v.to_le_bytes();
        let n = match op {
            StoreOp::Sb => 1,
            StoreOp::Sh => 2,
            StoreOp::Sw => 4,
        };
        buf[off..off + n].copy_from_slice(&b[..n]);
    }

    /// Начальное mscratch = 0. Шаги: csrrw v0; csrrs v1; csrrc v2; csrrwi 21;
    /// csrrsi 10; csrrci 5; csrrs x0 (только чтение); итоговое чтение.
    fn csr_sequence(&self, v: [u32; 3]) -> [u32; 8] {
        let mut s = 0u32;
        let mut out = [0u32; 8];
        out[0] = s;
        s = v[0];
        out[1] = s;
        s |= v[1];
        out[2] = s;
        s &= !v[2];
        out[3] = s;
        s = 21;
        out[4] = s;
        s |= 10;
        out[5] = s;
        s &= !5;
        out[6] = s;
        out[7] = s;
        out
    }

    fn upper(&self) -> [u32; 2] {
        [0xffff_f000, 0]
    }
}

/// Операнды для перебора: нули, единицы, границы знаковых/беззнаковых чисел,
/// значения около 32 (сдвиги), отрицательные.
pub const OPERANDS: [u32; 16] = [
    0,
    1,
    2,
    3,
    7,
    31,
    32,
    0x7fff_ffff,
    0x8000_0000,
    0x8000_0001,
    0xffff_ffff,
    0xffff_fffe,
    0xffff_fff9,
    0x1234_5678,
    0xdead_beef,
    0x0000_ffff,
];

/// Буфер для загрузок: байты со старшим битом и без.
pub const LOAD_PATTERN: [u8; 16] = [
    0x80, 0x7f, 0xff, 0x01, 0x00, 0x80, 0xfe, 0xff, 0x34, 0x12, 0x78, 0x56, 0xef, 0xbe, 0xad, 0xde,
];

/// Прогон команд через исполнитель; `out(ключ, значение)` для каждого результата.
pub fn run_isa<I: Isa>(isa: &I, out: &mut impl FnMut(&'static str, u32)) {
    for op in RrOp::ALL {
        let mut c = Crc32::new();
        for a in OPERANDS {
            for b in OPERANDS {
                c.update_u32(isa.rr(op, a, b));
            }
        }
        out(op.key(), c.finish());
    }
    // Граничные случаи по отдельности — чтобы расхождение было видно сразу.
    out("edge.div_by0", isa.rr(RrOp::Div, 7, 0));
    out("edge.divu_by0", isa.rr(RrOp::Divu, 7, 0));
    out("edge.rem_by0", isa.rr(RrOp::Rem, 7, 0));
    out("edge.remu_by0", isa.rr(RrOp::Remu, 7, 0));
    out("edge.div_ovf", isa.rr(RrOp::Div, 0x8000_0000, 0xffff_ffff));
    out("edge.rem_ovf", isa.rr(RrOp::Rem, 0x8000_0000, 0xffff_ffff));
    out("edge.div_neg", isa.rr(RrOp::Div, (-7i32) as u32, 2));
    out("edge.rem_neg", isa.rr(RrOp::Rem, (-7i32) as u32, 2));
    out("edge.mulh_minmin", isa.rr(RrOp::Mulh, 0x8000_0000, 0x8000_0000));
    out("edge.mulhsu_neg", isa.rr(RrOp::Mulhsu, 0xffff_ffff, 0xffff_ffff));
    out("edge.mulhu_max", isa.rr(RrOp::Mulhu, 0xffff_ffff, 0xffff_ffff));
    out("edge.sra_neg", isa.rr(RrOp::Sra, 0x8000_0000, 31));
    out("edge.sll_33", isa.rr(RrOp::Sll, 1, 33));

    for op in ImmOp::ALL {
        let mut c = Crc32::new();
        for a in OPERANDS {
            c.update_u32(isa.imm(op, a));
        }
        out(op.key(), c.finish());
    }

    let mut c = Crc32::new();
    for a in OPERANDS {
        for b in OPERANDS {
            c.update_u32(isa.branches(a, b));
        }
    }
    out("br.mask", c.finish());

    let loads: [(LoadOp, usize, &'static str); 5] = [
        (LoadOp::Lb, 1, "ld.lb"),
        (LoadOp::Lbu, 1, "ld.lbu"),
        (LoadOp::Lh, 2, "ld.lh"),
        (LoadOp::Lhu, 2, "ld.lhu"),
        (LoadOp::Lw, 4, "ld.lw"),
    ];
    for (op, step, key) in loads {
        let mut c = Crc32::new();
        let mut off = 0;
        while off < 16 {
            c.update_u32(isa.load(op, &LOAD_PATTERN, off));
            off += step;
        }
        out(key, c.finish());
    }

    let stores: [(StoreOp, usize, &'static str); 3] = [
        (StoreOp::Sb, 1, "st.sb"),
        (StoreOp::Sh, 2, "st.sh"),
        (StoreOp::Sw, 4, "st.sw"),
    ];
    for (op, step, key) in stores {
        let mut buf = [0u8; 16];
        let mut off = 0;
        let mut k = 0;
        while off < 16 {
            isa.store(op, &mut buf, off, OPERANDS[k % OPERANDS.len()].wrapping_mul(0x0101_0101));
            off += step;
            k += 3;
        }
        out(key, crc32(&buf));
    }

    let seq = isa.csr_sequence([0x1234_5678, 0x0000_ff00, 0x1030_0008]);
    let mut c = Crc32::new();
    for v in seq {
        c.update_u32(v);
    }
    out("csr.seq", c.finish());
    out("csr.seq_last", seq[7]);

    let up = isa.upper();
    out("upper.lui", up[0]);
    out("upper.auipc", up[1]);
}

/// Алгоритмы — одинаковый код на обеих платформах.
pub fn run_algo(out: &mut impl FnMut(&'static str, u32)) {
    out("crc32.check", crc32(black_box(b"123456789")));

    let mut r = XorShift32(black_box(0x2545_f491));
    let mut c = Crc32::new();
    for _ in 0..1024 {
        c.update_u32(r.next_u32());
    }
    out("crc32.prng4k", c.finish());

    out("fnv1a.empty", fnv1a32(black_box(b"")));
    out("fnv1a.text", fnv1a32(black_box(b"VMCU-C3 selftest")));

    let f90 = fib_u64(black_box(90));
    out("fib.90_lo", f90 as u32);
    out("fib.90_hi", (f90 >> 32) as u32);
    let mut c = Crc32::new();
    for n in 0..black_box(100) {
        let f = fib_u64(n);
        c.update_u32(f as u32);
        c.update_u32((f >> 32) as u32);
    }
    out("fib.crc", c.finish());

    let mut a = [0u32; 200];
    let mut r = XorShift32(black_box(12345));
    for v in a.iter_mut() {
        *v = r.next_u32();
    }
    heap_sort(&mut a);
    let sorted = a.windows(2).all(|w| w[0] <= w[1]) as u32;
    let mut c = Crc32::new();
    for v in a {
        c.update_u32(v);
    }
    out("sort.heap_crc", c.finish());
    out("sort.heap_ok", sorted);

    let mut b = [0i32; 64];
    let mut r = XorShift32(black_box(777));
    for v in b.iter_mut() {
        *v = r.next_u32() as i32 >> 7;
    }
    insertion_sort(&mut b);
    let mut c = Crc32::new();
    for v in b {
        c.update_u32(v as u32);
    }
    out("sort.insertion_crc", c.finish());

    let mut s = 0u64;
    for i in 0..black_box(500u64) {
        s = s.wrapping_add(isqrt_u64(i * i * 7919 + i));
    }
    out("isqrt.sum_lo", s as u32);

    out("f32.basel_1000", f32_basel(black_box(1000)).to_bits());
    let l = f64_leibniz(black_box(2000)).to_bits();
    out("f64.leibniz_lo", l as u32);
    out("f64.leibniz_hi", (l >> 32) as u32);
    let x = black_box(1.5f32);
    out("f32.div", (x / 7.0).to_bits());
    out("f32.mul_add", (x * 3.25 + 0.1).to_bits());
    out("f32.from_i32", (black_box(-123_456_789i32) as f32).to_bits());
    out("f32.to_i32", (black_box(-12345.678f32) as i32) as u32);
    out("f64.div", (black_box(2.0f64) / 3.0).to_bits() as u32);

    let d = u64_divmix(black_box(99), black_box(64));
    out("u64.divmix_lo", d as u32);
    out("u64.divmix_hi", (d >> 32) as u32);

    let m = matpow3(black_box([[1, 2, 3], [4, -5, 6], [-7, 8, 9]]), black_box(13));
    let mut c = Crc32::new();
    for row in m {
        for v in row {
            c.update_u32(v as u32);
        }
    }
    out("matpow.crc", c.finish());
}

/// Число ключей, которые выдают [`run_isa`] и [`run_algo`].
pub fn key_count() -> usize {
    let mut n = 0;
    run_isa(&RefIsa, &mut |_, _| n += 1);
    run_algo(&mut |_, _| n += 1);
    n
}
