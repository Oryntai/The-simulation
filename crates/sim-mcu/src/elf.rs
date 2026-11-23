//! Минимальный загрузчик ELF32 (little-endian, RISC-V, исполняемый файл).
//!
//! Свой, без сторонних crates: читаем заголовок, программные заголовки `PT_LOAD`
//! (что и куда класть в память) и таблицу символов `.symtab` (адреса переменных
//! прошивки — для тестов и отладки). Формат — System V ABI, ELF-32; поля RISC-V —
//! по «RISC-V ELF psABI».

use std::fmt;

/// Ошибка разбора ELF.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ElfError(pub String);

impl fmt::Display for ElfError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "ELF: {}", self.0)
    }
}

impl std::error::Error for ElfError {}

/// Загружаемый сегмент: байты файла кладутся по физическому адресу (LMA).
#[derive(Clone, Debug)]
pub struct Segment {
    /// Физический адрес (куда прошивальщик пишет байты; для `.data` — во флеше).
    pub paddr: u32,
    /// Виртуальный адрес (где сегмент живёт при исполнении).
    pub vaddr: u32,
    /// Содержимое из файла (`p_filesz` байт).
    pub data: Vec<u8>,
    /// Размер в памяти (`p_memsz`), хвост сверх `data` — нули (`.bss`).
    pub memsz: u32,
    /// Флаги `PF_X=1, PF_W=2, PF_R=4`.
    pub flags: u32,
}

/// Символ из `.symtab`.
#[derive(Clone, Debug)]
pub struct Symbol {
    pub name: String,
    pub value: u32,
    pub size: u32,
    /// Тип: 0 — NOTYPE, 1 — OBJECT, 2 — FUNC.
    pub kind: u8,
}

/// Разобранный образ.
#[derive(Clone, Debug)]
pub struct ElfImage {
    pub entry: u32,
    pub flags: u32,
    pub segments: Vec<Segment>,
    pub symbols: Vec<Symbol>,
}

const EM_RISCV: u16 = 243;
const ET_EXEC: u16 = 2;
const PT_LOAD: u32 = 1;
const SHT_SYMTAB: u32 = 2;
/// e_flags: бит 0 — есть сжатые инструкции, биты 1–2 — ABI плавающей точки, бит 3 — RVE.
const EF_RISCV_FLOAT_ABI: u32 = 0x6;
const EF_RISCV_RVE: u32 = 0x8;

fn err<T>(msg: impl Into<String>) -> Result<T, ElfError> {
    Err(ElfError(msg.into()))
}

fn u8_at(b: &[u8], off: usize) -> Result<u8, ElfError> {
    b.get(off)
        .copied()
        .ok_or_else(|| ElfError(format!("файл обрезан (смещение {off})")))
}

fn u16_at(b: &[u8], off: usize) -> Result<u16, ElfError> {
    match b.get(off..off.saturating_add(2)) {
        Some(s) => Ok(u16::from_le_bytes([s[0], s[1]])),
        None => err(format!("файл обрезан (смещение {off})")),
    }
}

fn u32_at(b: &[u8], off: usize) -> Result<u32, ElfError> {
    match b.get(off..off.saturating_add(4)) {
        Some(s) => Ok(u32::from_le_bytes([s[0], s[1], s[2], s[3]])),
        None => err(format!("файл обрезан (смещение {off})")),
    }
}

fn slice(b: &[u8], off: u32, len: u32) -> Result<&[u8], ElfError> {
    let (o, l) = (off as usize, len as usize);
    b.get(o..o.saturating_add(l)).ok_or_else(|| {
        ElfError(format!(
            "участок {o}+{l} за концом файла ({} байт)",
            b.len()
        ))
    })
}

/// Разбирает ELF32 RISC-V. Проверяет: магию, 32 бита, little-endian, машину RISC-V,
/// тип EXEC, мягкую плавающую точку (у нашего ядра нет F/D) и не RVE.
pub fn parse(b: &[u8]) -> Result<ElfImage, ElfError> {
    if b.len() < 52 || &b[0..4] != b"\x7fELF" {
        return err("нет сигнатуры \\x7fELF");
    }
    if u8_at(b, 4)? != 1 {
        return err("не 32-битный ELF (EI_CLASS ≠ ELFCLASS32)");
    }
    if u8_at(b, 5)? != 1 {
        return err("не little-endian (EI_DATA ≠ ELFDATA2LSB)");
    }
    let e_type = u16_at(b, 16)?;
    let e_machine = u16_at(b, 18)?;
    if e_machine != EM_RISCV {
        return err(format!("машина {e_machine}, ожидалась RISC-V ({EM_RISCV})"));
    }
    if e_type != ET_EXEC {
        return err(format!("тип {e_type}, ожидался исполняемый (ET_EXEC = 2)"));
    }
    let entry = u32_at(b, 24)?;
    let phoff = u32_at(b, 28)? as usize;
    let shoff = u32_at(b, 32)? as usize;
    let flags = u32_at(b, 36)?;
    if flags & EF_RISCV_FLOAT_ABI != 0 {
        return err("прошивка собрана под аппаратную плавающую точку, у ядра RV32IMC её нет");
    }
    if flags & EF_RISCV_RVE != 0 {
        return err("прошивка собрана под RV32E, ядро — RV32I");
    }
    let phentsize = u16_at(b, 42)? as usize;
    let phnum = u16_at(b, 44)? as usize;
    let shentsize = u16_at(b, 46)? as usize;
    let shnum = u16_at(b, 48)? as usize;

    let mut segments = Vec::new();
    if phnum > 0 && phentsize < 32 {
        return err("e_phentsize < 32");
    }
    for k in 0..phnum {
        let h = phoff + k * phentsize;
        if u32_at(b, h)? != PT_LOAD {
            continue;
        }
        let offset = u32_at(b, h + 4)?;
        let vaddr = u32_at(b, h + 8)?;
        let paddr = u32_at(b, h + 12)?;
        let filesz = u32_at(b, h + 16)?;
        let memsz = u32_at(b, h + 20)?;
        let pflags = u32_at(b, h + 24)?;
        if filesz > memsz {
            return err(format!("сегмент {k}: p_filesz > p_memsz"));
        }
        let data = slice(b, offset, filesz)?.to_vec();
        segments.push(Segment {
            paddr,
            vaddr,
            data,
            memsz,
            flags: pflags,
        });
    }
    if segments.is_empty() {
        return err("нет загружаемых сегментов PT_LOAD");
    }

    // Таблица символов (необязательна: файл может быть «обрезан» strip).
    let mut symbols = Vec::new();
    if shoff != 0 && shnum > 0 && shentsize >= 40 {
        for k in 0..shnum {
            let h = shoff + k * shentsize;
            if u32_at(b, h + 4)? != SHT_SYMTAB {
                continue;
            }
            let sym_off = u32_at(b, h + 16)?;
            let sym_size = u32_at(b, h + 20)?;
            let link = u32_at(b, h + 24)? as usize;
            let entsize = u32_at(b, h + 36)?.max(16) as usize;
            let sl = shoff + link * shentsize;
            let str_off = u32_at(b, sl + 16)?;
            let str_size = u32_at(b, sl + 20)?;
            let strtab = slice(b, str_off, str_size)?;
            let symtab = slice(b, sym_off, sym_size)?;
            for s in symtab.chunks_exact(entsize) {
                let name_off = u32::from_le_bytes([s[0], s[1], s[2], s[3]]) as usize;
                let value = u32::from_le_bytes([s[4], s[5], s[6], s[7]]);
                let size = u32::from_le_bytes([s[8], s[9], s[10], s[11]]);
                let kind = s[12] & 0xf;
                if name_off == 0 || name_off >= strtab.len() {
                    continue;
                }
                let end = strtab[name_off..]
                    .iter()
                    .position(|&c| c == 0)
                    .map_or(strtab.len(), |p| name_off + p);
                let name = String::from_utf8_lossy(&strtab[name_off..end]).into_owned();
                symbols.push(Symbol {
                    name,
                    value,
                    size,
                    kind,
                });
            }
        }
    }

    Ok(ElfImage {
        entry,
        flags,
        segments,
        symbols,
    })
}
