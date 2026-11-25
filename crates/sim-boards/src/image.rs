//! Загрузка прошивки: ELF32 (ARM, RISC-V, AVR), Intel HEX и raw-образы ESP32-C3.
//!
//! Формат ELF — System V ABI, ELF-32. Берём только то, что прошивальщик положил бы
//! в память: сегменты `PT_LOAD` с ненулевым содержимым по физическому адресу (LMA).
//! Формат HEX — «Hexadecimal Object File Format Specification», Intel, ред. A (1988);
//! контрольная сумма каждой записи проверяется.

use crate::fault::BoardFault;

#[path = "esp_image.rs"]
#[allow(
    dead_code,
    reason = "подключение raw-образов к BoardSim запланировано в Т2.4 и Т2.10"
)]
pub mod esp_image;

/// Архитектура образа (поле `e_machine`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Machine {
    Arm,
    RiscV,
    Avr,
}

impl Machine {
    fn code(self) -> u16 {
        match self {
            Machine::Arm => 40,
            Machine::RiscV => 243,
            Machine::Avr => 83,
        }
    }

    fn name(self) -> &'static str {
        match self {
            Machine::Arm => "ARM",
            Machine::RiscV => "RISC-V",
            Machine::Avr => "AVR",
        }
    }
}

/// Участок образа: что и куда записать.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Segment {
    pub addr: u32,
    pub data: Vec<u8>,
}

/// Разобранный образ.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Image {
    /// Точка входа из заголовка ELF (у HEX её нет).
    pub entry: Option<u32>,
    pub segments: Vec<Segment>,
}

fn bad<T>(s: impl Into<String>) -> Result<T, BoardFault> {
    Err(BoardFault::BadImage(s.into()))
}

fn u16_at(b: &[u8], o: usize) -> Result<u16, BoardFault> {
    match b.get(o..o.saturating_add(2)) {
        Some(s) => Ok(u16::from_le_bytes([s[0], s[1]])),
        None => bad(format!("ELF обрезан (смещение {o})")),
    }
}

fn u32_at(b: &[u8], o: usize) -> Result<u32, BoardFault> {
    match b.get(o..o.saturating_add(4)) {
        Some(s) => Ok(u32::from_le_bytes([s[0], s[1], s[2], s[3]])),
        None => bad(format!("ELF обрезан (смещение {o})")),
    }
}

/// Похоже ли на ELF.
pub fn is_elf(b: &[u8]) -> bool {
    b.len() >= 4 && &b[..4] == b"\x7fELF"
}

/// Разбирает исполняемый ELF32 little-endian нужной архитектуры.
pub fn parse_elf(b: &[u8], want: Machine) -> Result<Image, BoardFault> {
    if b.len() < 52 || !is_elf(b) {
        return bad("нет сигнатуры ELF");
    }
    if b[4] != 1 {
        return bad("не 32-битный ELF");
    }
    if b[5] != 1 {
        return bad("не little-endian");
    }
    if b[6] != 1 || u32_at(b, 20)? != 1 || u16_at(b, 40)? != 52 {
        return bad("неверная версия или размер заголовка ELF32");
    }
    let e_type = u16_at(b, 16)?;
    let e_machine = u16_at(b, 18)?;
    if e_machine != want.code() {
        return bad(format!(
            "прошивка для другой архитектуры: e_machine = {e_machine}, нужна {} ({})",
            want.name(),
            want.code()
        ));
    }
    if e_type != 2 {
        return bad(format!(
            "тип {e_type}, нужен исполняемый файл (ET_EXEC = 2)"
        ));
    }
    let entry = u32_at(b, 24)?;
    let phoff = u32_at(b, 28)? as usize;
    let phentsize = u16_at(b, 42)? as usize;
    let phnum = u16_at(b, 44)? as usize;
    if phnum == 0
        || phentsize != 32
        || phoff < 52
        || phoff
            .checked_add(phnum * phentsize)
            .is_none_or(|end| end > b.len())
    {
        return bad("нет программных заголовков");
    }
    let mut segments = Vec::new();
    for k in 0..phnum {
        let h = phoff.saturating_add(k.saturating_mul(phentsize));
        if u32_at(b, h)? != 1 {
            continue; // не PT_LOAD
        }
        let off = u32_at(b, h + 4)? as usize;
        let paddr = u32_at(b, h + 12)?;
        let filesz = u32_at(b, h + 16)? as usize;
        let memsz = u32_at(b, h + 20)? as usize;
        if (u64::from(paddr) + memsz as u64) > (1 << 32) {
            return bad(format!("сегмент {k}: адресный диапазон переполнен"));
        }
        if filesz > memsz {
            return bad(format!("сегмент {k}: p_filesz > p_memsz"));
        }
        if filesz == 0 {
            continue; // .bss: обнуляет сам код запуска
        }
        let data = match b.get(off..off.saturating_add(filesz)) {
            Some(d) => d.to_vec(),
            None => return bad(format!("сегмент {k} за концом файла")),
        };
        segments.push(Segment { addr: paddr, data });
    }
    if segments.is_empty() {
        return bad("нет загружаемых сегментов");
    }
    Ok(Image {
        entry: Some(entry),
        segments,
    })
}

fn hex_byte(s: &[u8], line: usize) -> Result<u8, BoardFault> {
    let d = |c: u8| (c as char).to_digit(16);
    match (
        s.first().copied().and_then(d),
        s.get(1).copied().and_then(d),
    ) {
        (Some(h), Some(l)) => Ok((h * 16 + l) as u8),
        _ => bad(format!("HEX, строка {line}: не шестнадцатеричная цифра")),
    }
}

/// Разбирает Intel HEX (записи 00, 01, 02, 03, 04, 05).
pub fn parse_ihex(b: &[u8]) -> Result<Image, BoardFault> {
    let text = match std::str::from_utf8(b) {
        Ok(t) => t,
        Err(_) => return bad("HEX: файл не текстовый"),
    };
    let mut base: u32 = 0;
    let mut segments: Vec<Segment> = Vec::new();
    let mut ended = false;
    for (idx, raw) in text.lines().enumerate() {
        let line_no = idx + 1;
        let line = raw.trim();
        if line.is_empty() {
            continue;
        }
        if ended {
            return bad(format!("HEX, строка {line_no}: данные после записи конца"));
        }
        let l = line.as_bytes();
        if l[0] != b':' || l.len() < 11 || l.len() % 2 == 0 {
            return bad(format!("HEX, строка {line_no}: неверный формат записи"));
        }
        let mut bytes = Vec::with_capacity((l.len() - 1) / 2);
        for c in l[1..].chunks(2) {
            bytes.push(hex_byte(c, line_no)?);
        }
        let n = bytes[0] as usize;
        if bytes.len() != n + 5 {
            return bad(format!("HEX, строка {line_no}: длина не совпадает"));
        }
        let sum = bytes.iter().fold(0u8, |a, &x| a.wrapping_add(x));
        if sum != 0 {
            return bad(format!("HEX, строка {line_no}: неверная контрольная сумма"));
        }
        let addr = u16::from_be_bytes([bytes[1], bytes[2]]) as u32;
        let data = &bytes[4..4 + n];
        match bytes[3] {
            0 if n != 0 => {
                let full = base
                    .checked_add(addr)
                    .filter(|full| u64::from(*full) + n as u64 <= 1 << 32)
                    .ok_or_else(|| {
                        BoardFault::BadImage(format!("HEX, строка {line_no}: переполнение адреса"))
                    })?;
                match segments.last_mut() {
                    Some(s) if s.addr.checked_add(s.data.len() as u32) == Some(full) => {
                        s.data.extend_from_slice(data)
                    }
                    _ => segments.push(Segment {
                        addr: full,
                        data: data.to_vec(),
                    }),
                }
            }
            1 if n == 0 && addr == 0 => ended = true,
            2 if n == 2 && addr == 0 => base = (u16::from_be_bytes([data[0], data[1]]) as u32) << 4,
            4 if n == 2 && addr == 0 => {
                base = (u16::from_be_bytes([data[0], data[1]]) as u32) << 16
            }
            3 | 5 if n == 4 && addr == 0 => {} // стартовый адрес: у AVR сброс всегда с нуля
            t => {
                return bad(format!(
                    "HEX, строка {line_no}: запись типа {t} недопустима"
                ));
            }
        }
    }
    if !ended {
        return bad("HEX: нет записи конца файла (файл обрезан?)");
    }
    if segments.is_empty() {
        return bad("HEX: нет данных");
    }
    Ok(Image {
        entry: None,
        segments,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hex_из_спецификации_читается() {
        let h = b":0300300002337A1E\n:00000001FF\n";
        let i = parse_ihex(h).unwrap();
        assert_eq!(i.segments[0].addr, 0x30);
        assert_eq!(i.segments[0].data, vec![0x02, 0x33, 0x7a]);
    }

    #[test]
    fn hex_с_битой_суммой_отклонён() {
        assert!(parse_ihex(b":0300300002337A1F\n:00000001FF\n").is_err());
    }

    #[test]
    fn hex_без_конца_отклонён() {
        assert!(parse_ihex(b":0300300002337A1E\n").is_err());
    }

    #[test]
    fn elf_чужой_архитектуры_отклонён() {
        let mut b = vec![0u8; 64];
        b[..4].copy_from_slice(b"\x7fELF");
        b[4] = 1;
        b[5] = 1;
        b[6] = 1;
        b[20] = 1;
        b[40] = 52;
        b[16] = 2;
        b[18] = 243;
        assert!(
            matches!(parse_elf(&b, Machine::Arm), Err(BoardFault::BadImage(s))
            if s.contains("архитектуры"))
        );
    }
}
