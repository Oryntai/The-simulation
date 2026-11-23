//! Что контроллер получает от мира и что ему отдаёт. Истинного состояния здесь нет.

use std::sync::Arc;

use crate::Time;

/// Номер выхода датчика или канала связи.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct SignalId(pub u32);

/// Номер входа привода или другого исполнительного канала.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ChannelId(pub u32);

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SignalSpec {
    pub name: String,
    pub unit: String,
}

/// Таблица имён. Порядок регистрации определяет номера и столбцы лога.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct SignalTable {
    specs: Vec<SignalSpec>,
}

impl SignalTable {
    /// Имена уникальны: два выхода с одним именем сделали бы лог неоднозначным.
    pub fn register(&mut self, name: &str, unit: &str) -> Result<SignalId, String> {
        if name.trim().is_empty() {
            return Err("пустое имя сигнала".into());
        }
        if self.id(name).is_some() {
            return Err(format!("сигнал «{name}» уже есть"));
        }
        let id = SignalId(u32::try_from(self.specs.len()).map_err(|_| "слишком много сигналов")?);
        self.specs.push(SignalSpec {
            name: name.to_owned(),
            unit: unit.to_owned(),
        });
        Ok(id)
    }

    pub fn id(&self, name: &str) -> Option<SignalId> {
        self.specs
            .iter()
            .position(|s| s.name == name)
            .map(|i| SignalId(i as u32))
    }

    pub fn spec(&self, id: SignalId) -> Option<&SignalSpec> {
        self.specs.get(id.0 as usize)
    }

    pub fn len(&self) -> usize {
        self.specs.len()
    }

    pub fn is_empty(&self) -> bool {
        self.specs.is_empty()
    }

    pub fn iter(&self) -> impl Iterator<Item = (SignalId, &SignalSpec)> {
        self.specs
            .iter()
            .enumerate()
            .map(|(i, s)| (SignalId(i as u32), s))
    }
}

/// Показание с моментом, когда оно стало доступно контроллеру.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sample {
    pub signal: SignalId,
    pub value: f64,
    pub at: Time,
    /// `false`: пропуск или отказ датчика, значение использовать нельзя.
    pub valid: bool,
}

/// Формат пикселей кадра.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PixelFormat {
    /// Три байта на пиксель, построчно сверху вниз.
    Rgb8,
    /// Глубина вдоль оптической оси, мм, u16 в порядке little-endian. Ноль означает дыру.
    DepthMm16,
}

impl PixelFormat {
    pub fn bytes_per_pixel(self) -> usize {
        match self {
            Self::Rgb8 => 3,
            Self::DepthMm16 => 2,
        }
    }
}

/// Кадр датчика. Пиксели общие между копиями, поэтому снимок мира не копирует изображения.
#[derive(Clone, Debug, PartialEq)]
pub struct ImageFrame {
    pub signal: SignalId,
    /// Начало экспозиции.
    pub captured: Time,
    /// Когда кадр становится доступен контроллеру.
    pub at: Time,
    pub width: u32,
    pub height: u32,
    pub format: PixelFormat,
    pub data: Arc<[u8]>,
    pub valid: bool,
}

impl ImageFrame {
    /// Размер данных соответствует разрешению и формату.
    pub fn is_consistent(&self) -> bool {
        (self.width as usize)
            .checked_mul(self.height as usize)
            .and_then(|n| n.checked_mul(self.format.bytes_per_pixel()))
            == Some(self.data.len())
    }

    /// Отпечаток содержимого, точно представимый в f64: для лога.
    pub fn fingerprint(&self) -> f64 {
        (crate::fnv1a(self.data.iter().copied()) >> 11) as f64
    }
}

/// Всё, что контроллер знает о мире в момент `time`.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Observation {
    pub time: Time,
    pub samples: Vec<Sample>,
    /// Последний кадр каждого датчика изображения.
    pub frames: Vec<ImageFrame>,
}

impl Observation {
    /// Последнее действительное показание сигнала.
    pub fn latest(&self, signal: SignalId) -> Option<f64> {
        self.samples
            .iter()
            .rev()
            .find(|s| s.signal == signal && s.valid)
            .map(|s| s.value)
    }

    /// Последний кадр сигнала.
    pub fn frame(&self, signal: SignalId) -> Option<&ImageFrame> {
        self.frames.iter().rev().find(|f| f.signal == signal)
    }
}

/// Команды исполнительным каналам.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Command {
    pub targets: Vec<(ChannelId, f64)>,
}
