use std::collections::BTreeMap;

use sim_boards::{BitOrder, I2cRequest, SpiMode, SpiRequest, TransferStatus};

use super::schema::{Table, tables};

#[derive(Clone, Debug)]
pub(super) enum Profile {
    I2c {
        address: u8,
        high: Vec<u8>,
        static_registers: BTreeMap<u8, u8>,
    },
    Spi {
        mode: SpiMode,
        address: u16,
        clear_error: u16,
        nop: u16,
        mask: u16,
        read: u16,
        parity: u16,
        error: u16,
        initial: u16,
    },
}

#[derive(Clone, Debug)]
pub(super) struct BusModel {
    pub unit: String,
    pub profile: Profile,
    pub source: String,
    codes: u16,
    period: f64,
    pointer: Option<u8>,
    next_word: u16,
    sticky_error: bool,
}

impl BusModel {
    pub fn parse(text: &str) -> Result<Self, String> {
        let mut tables = tables(text)?;
        let mut t = tables.remove(0);
        if t.number::<u32>("version")? != 1 {
            return Err(t.error("неподдержанная версия профиля"));
        }
        let unit = t.text("unit")?;
        let codes: u16 = t.number("codes")?;
        let period = t.finite("period")?;
        if !codes.is_power_of_two() || codes < 2 || period <= 0.0 {
            return Err(t.error("неверный масштаб регистра"));
        }
        let mut static_registers = BTreeMap::new();
        let profile = match t.text("kind")?.as_str() {
            "i2c" => {
                let address = t.number("address")?;
                let high = t.bytes("word_registers")?;
                if codes > (1 << 15) || address > 0x7f || high.is_empty() {
                    return Err(t.error("неверный адрес или размер регистра I²C"));
                }
                for mut register in tables {
                    if register.kind != "register" {
                        return Err(register.error("неизвестная таблица профиля"));
                    }
                    let addr = register.number("address")?;
                    let value = register.number("value")?;
                    register.source()?;
                    if static_registers.insert(addr, value).is_some() {
                        return Err(register.error("повтор регистра"));
                    }
                    register.finish()?;
                }
                let mut bytes: std::collections::BTreeSet<_> =
                    static_registers.keys().copied().collect();
                for &addr in &high {
                    if !bytes.insert(addr)
                        || !addr.checked_add(1).is_some_and(|byte| bytes.insert(byte))
                    {
                        return Err(t.error("пересечение регистров I²C"));
                    }
                }
                Profile::I2c {
                    address,
                    high,
                    static_registers,
                }
            }
            "spi" if tables.is_empty() => spi_profile(&mut t, codes)?,
            _ => return Err(t.error("неверный вид профиля шины")),
        };
        let source = t.source()?;
        t.finish()?;
        let next_word = match profile {
            Profile::Spi { initial, .. } => initial,
            _ => 0,
        };
        Ok(Self {
            unit,
            profile,
            source,
            codes,
            period,
            pointer: None,
            next_word,
            sticky_error: false,
        })
    }

    fn code(&self, value: f64) -> u16 {
        // Круговая шкала: k = floor(угол / полный оборот × число кодов).
        // min ограничивает ошибку округления у верхней границы f64.
        ((value.rem_euclid(self.period) / self.period * f64::from(self.codes)).floor() as u16)
            .min(self.codes - 1)
    }

    pub fn i2c(&mut self, request: &I2cRequest, sample: Option<f64>) -> (TransferStatus, Vec<u8>) {
        let Profile::I2c {
            address,
            high,
            static_registers,
        } = &self.profile
        else {
            unreachable!()
        };
        let Some(value) = sample else {
            return (TransferStatus::Nack, Vec::new());
        };
        if request.address != *address
            || request.write.len() > 1
            || (!request.write.is_empty() && request.read_len != 0 && !request.repeated_start)
        {
            return (TransferStatus::Nack, Vec::new());
        }
        let angle = self.code(value).to_be_bytes();
        let pointer = request.write.first().copied().or(self.pointer);
        let mut current = pointer;
        let mut bytes = Vec::new();
        for _ in 0..request.read_len {
            let Some(addr) = current else {
                return (TransferStatus::Nack, Vec::new());
            };
            let value = static_registers.get(&addr).copied().or_else(|| {
                high.iter().find_map(|&first| {
                    if addr == first {
                        Some(angle[0])
                    } else if addr == first + 1 {
                        Some(angle[1])
                    } else {
                        None
                    }
                })
            });
            let Some(value) = value else {
                return (TransferStatus::Nack, Vec::new());
            };
            bytes.push(value);
            // ams AS5600: указатель остаётся на старшем байте после чтения
            // младшего байта RAW ANGLE/ANGLE, для остальных регистров идёт дальше.
            current = high
                .iter()
                .find(|&&first| addr == first + 1)
                .copied()
                .or_else(|| addr.checked_add(1));
        }
        self.pointer = current;
        (TransferStatus::Complete, bytes)
    }

    pub fn spi(&mut self, request: &SpiRequest, sample: Option<f64>) -> (TransferStatus, Vec<u8>) {
        let Profile::Spi {
            mode,
            address,
            clear_error,
            nop,
            mask,
            read,
            parity,
            error,
            ..
        } = self.profile
        else {
            unreachable!()
        };
        let Some(value) = sample else {
            return (TransferStatus::NoResponse, Vec::new());
        };
        let [hi, lo] = request.write.as_slice() else {
            return (TransferStatus::NoResponse, Vec::new());
        };
        if request.mode != mode
            || request.write_order != BitOrder::MsbFirst
            || request.read_order != BitOrder::MsbFirst
        {
            return (TransferStatus::NoResponse, Vec::new());
        }
        let input = u16::from_be_bytes([*hi, *lo]);
        let register = input & mask;
        let response = self.next_word.to_be_bytes().to_vec();
        let payload = if !input.count_ones().is_multiple_of(2) {
            self.sticky_error = true;
            0
        } else if input & read != 0 && register == address {
            self.code(value)
        } else if input & read != 0 && register == clear_error {
            // Флаг ошибки относится к предыдущей команде. Детализация причин
            // в отдельном регистре в этой модели отсутствует.
            self.sticky_error = false;
            0
        } else if input == nop {
            0
        } else {
            self.sticky_error = true;
            0
        };
        let word = payload | if self.sticky_error { error } else { 0 };
        self.next_word = if word.count_ones().is_multiple_of(2) {
            word
        } else {
            word | parity
        };
        (TransferStatus::Complete, response)
    }
}

fn spi_profile(t: &mut Table, codes: u16) -> Result<Profile, String> {
    let mode = match t.number::<u8>("mode")? {
        0 => SpiMode::Mode0,
        1 => SpiMode::Mode1,
        2 => SpiMode::Mode2,
        3 => SpiMode::Mode3,
        _ => return Err(t.error("неверный режим SPI")),
    };
    let address = t.number("address")?;
    let clear_error = t.number("clear_error")?;
    let nop = t.number("nop")?;
    let mask: u16 = t.number("mask")?;
    let read: u16 = t.number("read_bit")?;
    let parity: u16 = t.number("parity_bit")?;
    let error: u16 = t.number("error_bit")?;
    let initial: u16 = t.number("initial_word")?;
    if ![read, parity, error].iter().all(|v| v.is_power_of_two())
        || read == parity
        || error == parity
        || (read | parity | error) & mask != 0
        || address & !mask != 0
        || clear_error & !mask != 0
        || clear_error == address
        || (codes - 1) & !mask != 0
        || !initial.count_ones().is_multiple_of(2)
    {
        return Err(t.error("пересечение полей слова SPI"));
    }
    Ok(Profile::Spi {
        mode,
        address,
        clear_error,
        nop,
        mask,
        read,
        parity,
        error,
        initial,
    })
}
