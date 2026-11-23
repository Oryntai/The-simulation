//! Идентификаторы выводов совпадают с интерфейсом симуляции.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BoardId {
    Esp32C3,
    Rp2040,
    Stm32f103c8,
    Atmega328p,
    Virtual,
}

impl BoardId {
    pub const ALL: [Self; 5] = [
        Self::Esp32C3,
        Self::Rp2040,
        Self::Stm32f103c8,
        Self::Atmega328p,
        Self::Virtual,
    ];

    pub const fn key(self) -> &'static str {
        match self {
            Self::Esp32C3 => "esp32-c3",
            Self::Rp2040 => "rp2040",
            Self::Stm32f103c8 => "stm32f103c8",
            Self::Atmega328p => "atmega328p",
            Self::Virtual => "vmcu-c3",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Range {
    pub min: f64,
    pub max: f64,
}

impl Range {
    pub(crate) fn contains(self, value: f64) -> bool {
        value.is_finite() && value >= self.min && value <= self.max
    }
}

const fn range(min: f64, max: f64) -> Range {
    Range { min, max }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SupplyRail {
    Mcu,
    Vsys,
    Vin,
    Usb,
}

#[derive(Clone, Copy, Debug)]
pub struct BoardSpec {
    pub id: BoardId,
    pub name: &'static str,
    pub mcu_supply_v: Range,
    pub abs_mcu_supply_v: Range,
    pub nominal_io_v: f64,
    pub vil_max_v: f64,
    pub vih_min_v: f64,
    pub vol_max_v: f64,
    pub voh_min_v: f64,
    /// Суммарный предел собственно цифровых выводов, если указан отдельно от питания.
    pub gpio_total_ma: Option<f64>,
    pub abs_total_ma: Option<f64>,
    pub adc_bits: u8,
    /// Верхняя граница диапазона АЦП при любой штатной настройке.
    pub adc_max_v: f64,
    pub source: &'static str,
}

#[derive(Clone, Copy, Debug)]
pub struct PinSpec {
    pub id: u32,
    pub gpio: bool,
    pub pwm: bool,
    pub adc_channel: Option<u8>,
    pub uart: bool,
    pub i2c: bool,
    pub spi: bool,
    pub five_volt_input: bool,
    pub strap: bool,
    pub reserved: Option<&'static str>,
    pub source_ma: Option<f64>,
    pub sink_ma: Option<f64>,
    pub abs_pin_ma: Option<f64>,
    /// Абсолютные пределы при номинальном VDD, если даташит их указывает.
    pub abs_input_v: Option<Range>,
    /// Например, 40/28 мА типовых для ESP32-C3 при максимальном DRIVE.
    pub typical_drive_ma: Option<(f64, f64)>,
    pub internal_only: bool,
    pub digital_input_max_v: f64,
}

pub fn board_spec(id: BoardId) -> BoardSpec {
    use BoardId::*;
    match id {
        // Масштаб идеального АЦП и двоичный интерфейс модели. Не паспортные пределы.
        Virtual => {
            let vref = sim_mcu::periph::adc::VREF;
            BoardSpec {
                id,
                name: "VMCU-C3",
                mcu_supply_v: range(vref, vref),
                abs_mcu_supply_v: range(vref, vref),
                nominal_io_v: vref,
                vil_max_v: 0.0,
                vih_min_v: vref,
                vol_max_v: 0.0,
                voh_min_v: vref,
                gpio_total_ma: None,
                abs_total_ma: None,
                adc_bits: sim_mcu::periph::adc::BITS as u8,
                adc_max_v: vref,
                source: "crates/sim-mcu/MEMORY_MAP.md",
            }
        }
        // ESP32-C3 Datasheet, таблицы 5-1, 5-2, 5-4, 5-5.
        Esp32C3 => BoardSpec {
            id,
            name: "ESP32-C3",
            mcu_supply_v: range(3.0, 3.6),
            abs_mcu_supply_v: range(-0.3, 3.6),
            nominal_io_v: 3.3,
            vil_max_v: 0.25 * 3.3,
            vih_min_v: 0.75 * 3.3,
            vol_max_v: 0.1 * 3.3,
            voh_min_v: 0.8 * 3.3,
            gpio_total_ma: None,
            abs_total_ma: Some(1000.0),
            adc_bits: 12,
            adc_max_v: 2.5,
            source: "https://documentation.espressif.com/esp32-c3_datasheet_en.html",
        },
        // RP2040 Datasheet, разделы 2.9.1, 4.9, 5.2; Pico Datasheet §2.
        Rp2040 => BoardSpec {
            id,
            name: "Raspberry Pi Pico",
            mcu_supply_v: range(1.8, 3.3),
            abs_mcu_supply_v: range(-0.5, 3.63),
            nominal_io_v: 3.3,
            vil_max_v: 0.8,
            vih_min_v: 2.0,
            vol_max_v: 0.5,
            voh_min_v: 2.62,
            gpio_total_ma: Some(50.0),
            abs_total_ma: None,
            adc_bits: 12,
            adc_max_v: 3.3,
            source: "https://pip-assets.raspberrypi.com/categories/814-rp2040/documents/RP-008371-DS-1-rp2040-datasheet.pdf",
        },
        // DS5319, таблицы 6, 7, 9, 36, 46; пороги CMOS при номинальных 3,3 В.
        Stm32f103c8 => BoardSpec {
            id,
            name: "STM32F103C8 (Blue Pill)",
            mcu_supply_v: range(2.0, 3.6),
            abs_mcu_supply_v: range(-0.3, 4.0),
            nominal_io_v: 3.3,
            vil_max_v: 0.35 * 3.3,
            vih_min_v: 0.65 * 3.3,
            vol_max_v: 0.4,
            voh_min_v: 3.3 - 0.4,
            gpio_total_ma: None,
            abs_total_ma: Some(150.0),
            adc_bits: 12,
            adc_max_v: 3.6,
            source: "https://www.st.com/resource/en/datasheet/stm32f103c8.pdf",
        },
        // ATmega328P 7810D, §§28.1–28.3, 24; платы Arduino работают на 16 МГц.
        Atmega328p => BoardSpec {
            id,
            name: "ATmega328P (Uno/Nano)",
            mcu_supply_v: range(4.5, 5.5),
            abs_mcu_supply_v: range(-0.5, 6.0),
            nominal_io_v: 5.0,
            vil_max_v: 0.3 * 5.0,
            vih_min_v: 0.6 * 5.0,
            vol_max_v: 0.8,
            voh_min_v: 4.1,
            gpio_total_ma: None,
            abs_total_ma: Some(200.0),
            adc_bits: 10,
            adc_max_v: 5.5,
            source: "https://ww1.microchip.com/downloads/aemDocuments/documents/MCU08/ProductDocuments/DataSheets/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf",
        },
    }
}

pub fn supply_range(id: BoardId, rail: SupplyRail) -> Option<Range> {
    match (id, rail) {
        (BoardId::Virtual, _) => None,
        (_, SupplyRail::Mcu) => Some(board_spec(id).mcu_supply_v),
        // Pico Datasheet §3.4, для отдельного источника без USB.
        (BoardId::Rp2040, SupplyRail::Vsys) => Some(range(1.8, 5.5)),
        // USB 2.0 §7.2.1, диапазон VBUS у источника high-power.
        (BoardId::Rp2040 | BoardId::Atmega328p, SupplyRail::Usb) => Some(range(4.75, 5.25)),
        // Документация Arduino, Powering Alternatives, Uno Rev3 и классический Nano.
        (BoardId::Atmega328p, SupplyRail::Vin) => Some(range(7.0, 12.0)),
        _ => None,
    }
}

/// Номинальные выводы: Pico GP; ESP GPIO; STM PA0=0, PB0=16, PC0=32;
/// STM BOOT0=64; Arduino D0…D13, A0…A7 = 14…21 (A6/A7 есть только на Nano).
pub fn pin_spec(board: BoardId, id: u32) -> Option<PinSpec> {
    let mut p = PinSpec {
        id,
        gpio: true,
        pwm: false,
        adc_channel: None,
        uart: false,
        i2c: false,
        spi: false,
        five_volt_input: false,
        strap: false,
        reserved: None,
        source_ma: None,
        sink_ma: None,
        abs_pin_ma: None,
        abs_input_v: None,
        typical_drive_ma: None,
        internal_only: false,
        digital_input_max_v: board_spec(board).nominal_io_v,
    };
    match board {
        // GPIO и аналоговые каналы имеют отдельные пространства номеров.
        BoardId::Virtual if id < 32 => {
            p.pwm = (id as usize) < sim_mcu::periph::pwm::CHANNELS;
        }
        BoardId::Esp32C3 if id < 22 => {
            // Datasheet, таблицы 2-1, 2-4, 2-6, 3-1, 5-4; 40/28 мА — типовые.
            p.pwm = true;
            p.i2c = true;
            p.uart = true;
            p.spi = true;
            if id < 6 {
                p.adc_channel = Some(id as u8);
            }
            p.strap = matches!(id, 2 | 8 | 9);
            // Консервативная разводка: линии flash и USB оставлены за их штатными функциями.
            p.reserved = match id {
                11 => Some("GPIO11 используется как VDD_SPI в стандартной разводке"),
                12..=17 => Some("шина флеш-памяти или вывод отсутствует"),
                18 | 19 => Some("USB Serial/JTAG"),
                _ => None,
            };
            p.internal_only = matches!(id, 11..=17);
            p.typical_drive_ma = Some((40.0, 28.0));
        }
        BoardId::Rp2040 if id < 30 => {
            // RP2040 Datasheet §2.19.2, §5.2; Pico Datasheet, схема и pinout.
            p.pwm = true;
            p.i2c = true;
            p.spi = true;
            p.uart = true; // UART TX/RX/CTS/RTS в таблице мультиплексора.
            p.adc_channel = (id >= 26).then(|| (id - 26) as u8);
            p.reserved = match id {
                23 => Some("внутреннее управление режимом преобразователя Pico"),
                24 => Some("внутреннее измерение VBUS Pico"),
                25 => Some("бортовой светодиод Pico"),
                29 => Some("внутренний делитель VSYS Pico"),
                _ => None,
            };
            // Минимальная выбираемая ступень DRIVE по таблице PADS_BANK0; оценка для неизвестной настройки.
            p.source_ma = Some(2.0);
            p.sink_ma = p.source_ma;
            p.internal_only = matches!(id, 23..=25 | 29);
            p.abs_input_v = Some(range(-0.5, 3.3 + 0.5));
        }
        BoardId::Stm32f103c8 if id < 32 || (45..=47).contains(&id) => {
            // DS5319, выводы LQFP48 (основные и альтернативные функции с AFIO remap).
            p.adc_channel = match id {
                0..=7 => Some(id as u8),
                16..=17 => Some((id - 8) as u8),
                _ => None,
            };
            p.pwm = matches!(id, 0..=3 | 6..=11 | 15..=17 | 19..=27 | 29..=31);
            p.uart = matches!(id, 0..=4 | 8..=12 | 22..=23 | 26..=30);
            p.i2c = matches!(id, 21..=28);
            p.spi = matches!(id, 4..=7 | 15 | 19..=21 | 28..=31);
            p.five_volt_input = matches!(id, 8..=15 | 18..=20 | 22..=31);
            p.strap = id == 18; // BOOT1 (PB2); BOOT0 — отдельный вывод.
            p.reserved = match id {
                13..=15 | 19..=20 => Some("SWD/JTAG при сбросе"),
                46 | 47 => Some("выводы низкочастотного кварца Blue Pill"),
                _ => None,
            };
            p.source_ma = Some(if id >= 45 { 0.0 } else { 8.0 });
            p.sink_ma = Some(if id >= 45 { 3.0 } else { 8.0 });
            p.abs_pin_ma = Some(if id >= 45 { 3.0 } else { 25.0 });
            p.abs_input_v = Some(range(-0.3, if p.five_volt_input { 3.3 + 4.0 } else { 4.0 }));
            if p.five_volt_input {
                p.digital_input_max_v = 5.0;
            }
        }
        // DS5319 таблица 5, BOOT0 — выделенный вход. В каталоге ID 64.
        BoardId::Stm32f103c8 if id == 64 => {
            p.gpio = false;
            p.strap = true;
            p.five_volt_input = true;
            p.digital_input_max_v = 5.5;
            p.reserved = Some("BOOT0, выделенный вход режима загрузки");
        }
        BoardId::Atmega328p if id < 22 => {
            // Arduino Uno/Nano pinout; 7810D, §§1, 28.1, 28.2.
            p.gpio = id < 20;
            p.pwm = matches!(id, 3 | 5 | 6 | 9 | 10 | 11);
            p.adc_channel = (id >= 14).then(|| (id - 14) as u8);
            p.uart = id < 2;
            p.i2c = matches!(id, 18 | 19);
            p.spi = (10..=13).contains(&id);
            p.five_volt_input = true; // При питании 5 В, включая аналоговые входы Nano.
            p.reserved = (id >= 20).then_some("A6/A7 только на Nano, исключительно АЦП");
            p.source_ma = p.gpio.then_some(20.0);
            p.sink_ma = p.source_ma;
            p.abs_pin_ma = p.gpio.then_some(40.0);
            p.abs_input_v = Some(range(-0.5, 5.0 + 0.5));
        }
        _ => return None,
    }
    Some(p)
}

/// Все выводы, представленные в каталоге, в порядке идентификаторов.
pub fn board_pins(id: BoardId) -> Vec<PinSpec> {
    let upper = match id {
        BoardId::Stm32f103c8 => 64, // BOOT0.
        BoardId::Virtual => 31,
        BoardId::Rp2040 => 29,
        BoardId::Esp32C3 | BoardId::Atmega328p => 21,
    };
    (0..=upper).filter_map(|pin| pin_spec(id, pin)).collect()
}

/// Позиция каталога. Ключ позиции не выбирает карту памяти напрямую.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BoardEntry {
    pub id: String,
    pub profile: BoardId,
    pub name: String,
    pub source: String,
}

#[derive(Clone, Debug)]
pub struct BoardCatalog(Vec<BoardEntry>);

impl BoardCatalog {
    /// Строгая схема `[[board]]`: четыре обязательные строки без escape-последовательностей.
    pub fn parse(input: &str) -> Result<Self, crate::BoardFault> {
        let mut entries = Vec::new();
        let mut fields = std::collections::BTreeMap::new();
        let mut table = false;
        for (index, line) in input.lines().enumerate() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            if line == "[[board]]" {
                if table {
                    parse_entry(&mut fields, &mut entries)?;
                }
                table = true;
                continue;
            }
            let field = || catalog_error(format!("строка {}: неверное поле", index + 1));
            let (key, raw) = line.split_once('=').ok_or_else(field)?;
            let key = key.trim();
            let (value, tail) = raw
                .trim_start()
                .strip_prefix('"')
                .and_then(|s| s.split_once('"'))
                .ok_or_else(field)?;
            let tail = tail.trim();
            if !table
                || !matches!(key, "id" | "profile" | "name" | "source")
                || value.trim().is_empty()
                || value.contains(|c: char| c.is_control() || c == '\\')
                || !(tail.is_empty() || tail.starts_with('#'))
                || fields.insert(key, value).is_some()
            {
                return Err(field());
            }
        }
        if !table {
            return Err(catalog_error("нет ни одной позиции"));
        }
        parse_entry(&mut fields, &mut entries)?;
        Ok(Self(entries))
    }

    pub fn builtin() -> Result<Self, crate::BoardFault> {
        Self::parse(include_str!("../../../catalog/boards/catalog.toml"))
    }

    pub fn entries(&self) -> &[BoardEntry] {
        &self.0
    }

    pub fn get(&self, id: &str) -> Option<&BoardEntry> {
        self.0.iter().find(|entry| entry.id == id)
    }

    pub fn create(&self, id: &str) -> Result<Box<dyn crate::BoardSim>, crate::BoardFault> {
        let entry = self
            .get(id)
            .ok_or_else(|| catalog_error(format!("неизвестная позиция {id}")))?;
        Ok(crate::new_board(entry.profile))
    }
}

fn catalog_error(message: impl std::fmt::Display) -> crate::BoardFault {
    crate::BoardFault::Misconfigured(format!("каталог плат: {message}"))
}

fn parse_entry(
    fields: &mut std::collections::BTreeMap<&str, &str>,
    entries: &mut Vec<BoardEntry>,
) -> Result<(), crate::BoardFault> {
    let mut take = |key| {
        fields
            .remove(key)
            .ok_or_else(|| catalog_error(format!("нет поля {key}")))
    };
    let id = take("id")?;
    let profile_key = take("profile")?;
    let profile = BoardId::ALL
        .into_iter()
        .find(|p| p.key() == profile_key)
        .ok_or_else(|| catalog_error(format!("неизвестный профиль {profile_key}")))?;
    if !id
        .bytes()
        .all(|b| b.is_ascii_lowercase() || b.is_ascii_digit() || b == b'-')
        || entries.iter().any(|entry| entry.id == id)
    {
        return Err(catalog_error(format!(
            "неверный или повторный ключ позиции {id}"
        )));
    }
    entries.push(BoardEntry {
        id: id.into(),
        profile,
        name: take("name")?.into(),
        source: take("source")?.into(),
    });
    Ok(())
}
