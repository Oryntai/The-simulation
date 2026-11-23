//! GPIO: 32 вывода.
//!
//! | смещение | регистр | доступ |
//! |---|---|---|
//! | 0x00 | OUT — выходные уровни | RW |
//! | 0x04 | OUT_SET — записанные единицы ставят биты OUT | W |
//! | 0x08 | OUT_CLR — записанные единицы сбрасывают биты OUT | W |
//! | 0x0C | OUT_TOGGLE — записанные единицы инвертируют биты OUT | W |
//! | 0x10 | DIR — 1 = выход | RW |
//! | 0x14 | IN — уровни на выводах: выход → OUT, вход → уровень от хоста | R |
//!
//! После сброса все выводы — входы (Hi-Z), OUT = 0. Входы от хоста по умолчанию
//! в высоком уровне (подтяжка вверх).

#[derive(Clone, Debug)]
pub struct Gpio {
    pub out: u32,
    pub dir: u32,
    /// Уровни, которые задаёт хост (внешний мир).
    pub input: u32,
}

impl Default for Gpio {
    fn default() -> Self {
        Gpio {
            out: 0,
            dir: 0,
            input: u32::MAX,
        }
    }
}

impl Gpio {
    pub fn reset(&mut self) {
        self.out = 0;
        self.dir = 0;
    }

    /// Уровни на выводах.
    pub fn levels(&self) -> u32 {
        (self.out & self.dir) | (self.input & !self.dir)
    }

    pub fn read(&self, reg: u32) -> Option<u32> {
        Some(match reg {
            0x00 => self.out,
            0x10 => self.dir,
            0x14 => self.levels(),
            0x04 | 0x08 | 0x0c => 0,
            _ => return None,
        })
    }

    pub fn write(&mut self, reg: u32, v: u32) -> bool {
        match reg {
            0x00 => self.out = v,
            0x04 => self.out |= v,
            0x08 => self.out &= !v,
            0x0c => self.out ^= v,
            0x10 => self.dir = v,
            _ => return false,
        }
        true
    }
}
