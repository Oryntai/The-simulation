//! Явные входы для неизвестных состояний сброса и наблюдений на шинах ESP32-C3.

use crate::BoardFault;
use std::{collections::BTreeMap, sync::Arc};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EspResetValueKind {
    Qemu,
    SiliconMeasurement,
    Doc,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EspResetValue {
    pub address: u32,
    pub field: String,
    pub value: u32,
    pub source: String,
    pub kind: EspResetValueKind,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EspResetValueUse {
    pub entry: EspResetValue,
    /// Число подстановок при успешно завершённых чтениях.
    pub reads: u64,
    /// Фактические адреса чтения и счётчики. У записи на всю память
    /// здесь адреса отдельных слов, у MMIO — адрес поля из записи.
    pub read_addresses: BTreeMap<u32, u64>,
}

#[derive(Clone, Debug)]
pub struct EspResetProfile(Arc<[EspResetValue]>);

impl EspResetProfile {
    /// Строгая схема TOML `[[reset]]`: два целых и три строки.
    /// Строки задаются в двойных кавычках без escape-последовательностей.
    pub fn parse(input: &str) -> Result<Self, BoardFault> {
        let mut entries = Vec::new();
        let mut fields = BTreeMap::new();
        let mut in_table = false;
        for (index, line) in input.lines().enumerate() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            if line == "[[reset]]" {
                if in_table {
                    entries.push(take_entry(&mut fields)?);
                }
                in_table = true;
                continue;
            }
            let error = || invalid(format!("неверное поле в строке {}", index + 1));
            let (key, literal) = line.split_once('=').ok_or_else(error)?;
            let key = key.trim();
            if !in_table
                || !matches!(key, "address" | "field" | "value" | "source" | "kind")
                || fields.insert(key, literal.trim()).is_some()
            {
                return Err(error());
            }
        }
        if !in_table {
            return Err(invalid("нет записей [[reset]]"));
        }
        entries.push(take_entry(&mut fields)?);
        entries.sort_by(|a, b| (a.address, &a.field).cmp(&(b.address, &b.field)));
        if entries
            .windows(2)
            .any(|pair| (pair[0].address, &pair[0].field) == (pair[1].address, &pair[1].field))
        {
            return Err(invalid("повтор адреса и поля"));
        }
        Ok(Self(entries.into()))
    }

    pub fn entries(&self) -> &[EspResetValue] {
        &self.0
    }
}

fn invalid(detail: impl std::fmt::Display) -> BoardFault {
    BoardFault::Misconfigured(format!("профиль сброса ESP32-C3: {detail}"))
}

fn take_entry(fields: &mut BTreeMap<&str, &str>) -> Result<EspResetValue, BoardFault> {
    let mut take = |key| {
        fields
            .remove(key)
            .ok_or_else(|| invalid(format!("нет {key}")))
    };
    let address = parse_int(take("address")?)?;
    let field = parse_string(take("field")?)?;
    let value = parse_int(take("value")?)?;
    let source = parse_string(take("source")?)?;
    let kind = match parse_string(take("kind")?)? {
        "qemu" => EspResetValueKind::Qemu,
        "silicon_measurement" => EspResetValueKind::SiliconMeasurement,
        "doc" => EspResetValueKind::Doc,
        _ => return Err(invalid("неизвестный kind")),
    };
    if !crate::peripheral::supports_reset_value(address, field, value)
        && !crate::esp_memory::supports_reset_value(address, field)
        && !crate::peripheral::esp_sleep::supports_crc_profile(address, field, value, kind)
    {
        return Err(invalid(format!(
            "недопустимое значение для {address:#010x} {field}"
        )));
    }
    Ok(EspResetValue {
        address,
        field: field.into(),
        value,
        source: source.into(),
        kind,
    })
}

fn parse_int(literal: &str) -> Result<u32, BoardFault> {
    let word = literal.split('#').next().unwrap_or_default().trim();
    let result = if let Some(hex) = word.strip_prefix("0x") {
        u32::from_str_radix(hex, 16)
    } else {
        word.parse()
    };
    result.map_err(|_| invalid("требуется целое u32"))
}

fn parse_string(literal: &str) -> Result<&str, BoardFault> {
    let (value, tail) = literal
        .strip_prefix('"')
        .and_then(|text| text.split_once('"'))
        .ok_or_else(|| invalid("требуется строка в двойных кавычках"))?;
    let tail = tail.trim_start();
    if value.trim().is_empty()
        || value.contains(|c: char| c.is_control() || c == '\\')
        || !(tail.is_empty() || tail.starts_with('#'))
    {
        return Err(invalid("недопустимая строка"));
    }
    Ok(value)
}

#[derive(Clone, Default)]
pub(crate) struct EspResetValues {
    profile: Option<EspResetProfile>,
    used: Vec<EspResetValueUse>,
}

impl EspResetValues {
    pub(crate) fn new(profile: Option<EspResetProfile>) -> Self {
        Self {
            profile,
            used: Vec::new(),
        }
    }

    pub(crate) fn usage(&self) -> &[EspResetValueUse] {
        &self.used
    }

    pub(crate) fn restore_usage(&mut self, used: Vec<EspResetValueUse>) {
        self.used = used;
    }

    pub(crate) fn take_usage(&mut self) -> Vec<EspResetValueUse> {
        std::mem::take(&mut self.used)
    }

    pub(crate) fn read(&mut self, address: u32, field: &str) -> Result<Option<u32>, BoardFault> {
        self.read_at(address, field, address)
    }

    pub(crate) fn contains(&self, address: u32, field: &str) -> bool {
        Self::entry(self.profile.as_ref(), address, field).is_some()
    }

    pub(crate) fn read_at(
        &mut self,
        address: u32,
        field: &str,
        read_address: u32,
    ) -> Result<Option<u32>, BoardFault> {
        let key = (address, field);
        let Some(entry) = Self::entry(self.profile.as_ref(), address, field) else {
            return Ok(None);
        };
        let value = entry.value;
        match self
            .used
            .binary_search_by_key(&key, |used| (used.entry.address, used.entry.field.as_str()))
        {
            Ok(index) => {
                let used = &mut self.used[index];
                used.reads = used.reads.checked_add(1).ok_or_else(|| {
                    BoardFault::Execution("переполнение счётчика чтений профиля сброса".into())
                })?;
                *used.read_addresses.entry(read_address).or_default() += 1;
            }
            Err(index) => self.used.insert(
                index,
                EspResetValueUse {
                    entry: entry.clone(),
                    reads: 1,
                    read_addresses: BTreeMap::from([(read_address, 1)]),
                },
            ),
        }
        Ok(Some(value))
    }

    fn entry<'a>(
        profile: Option<&'a EspResetProfile>,
        address: u32,
        field: &str,
    ) -> Option<&'a EspResetValue> {
        let entries = profile?.entries();
        let index = entries
            .binary_search_by_key(&(address, field), |entry| {
                (entry.address, entry.field.as_str())
            })
            .ok()?;
        Some(&entries[index])
    }
}
