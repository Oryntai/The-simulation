use std::collections::{BTreeMap, BTreeSet};
use std::str::FromStr;

use sim_control::{Delay, InputKind};
use sim_process::schematic::mcu_energy::{McuCurrent, McuCurrentKind, McuPowerProfile};
use sim_radio::timed::{
    AttemptLink, EspNowConfirmation, EspNowInterface, EspNowLink, EspNowPeer, EspNowProfile,
    EspNowVersion,
};
use sim_signal::{PixelFormat, Rate, Time};
use sim_world::Provenance;

/// Документ версии 1. Неизвестные, повторные и неприменимые поля отклоняются.
/// `[[board]]` использует прежнюю загрузку ELF/HEX. `catalog_asset` задаёт свой
/// каталог, а `flash_profile`, `rom` и `efuse` вместе включают загрузку raw flash.
/// `power_profile` и `power_schedule` задают точки тока и график расчёта заряда.
/// `[[external_board]]` объявляет диагностическую плату и её `profile`, с
/// необязательными `image` и `efuse` QEMU. Все ссылки разрешаются относительно графа.
/// У `[[packet]] kind = "esp_now"` `profile_asset` задаёт адресата, каналы,
/// правило подтверждения, длительность попытки и модель потерь.
/// В профиле `format = 1`, `kind = "esp_now"` обязательны `sender_version`,
/// `wifi_channel`, `station_enabled`, `soft_ap_enabled`, `queue_capacity`,
/// `attempt_ns`, `destination` (шесть октетов), `interface` (station или
/// soft_ap), `peer_channel`, `encrypted`, `loss`, `rssi_dbm`, `source`.
/// Unicast использует `mac_attempts` и `mac_ack_loss`. Без `receiver_channel`
/// на линии нет приёмника. `receiver_version` нужен для длинных v2-пакетов.
/// При `confirmation = "application"` нужны `timeout_ns` и `transmissions`.
/// Путь ответа задаётся вместе полями `reply_attempt_ns`, `reply_loss`,
/// `reply_rssi_dbm`. При его отсутствии вызов ACK виден как пропуск ответа.
#[derive(Clone, Debug)]
pub struct ControlConfig {
    pub(super) text: String,
    pub(super) tables: Vec<Table>,
}

impl ControlConfig {
    pub fn parse(text: &str) -> Result<Self, String> {
        let mut tables = tables(text)?;
        let mut header = tables.remove(0);
        if header.number::<u32>("version")? != 1 {
            return Err("неподдержанная версия control.toml".into());
        }
        header.finish()?;
        let boards = tables
            .iter()
            .filter(|t| t.kind == "board")
            .map(|t| board(t.clone()))
            .collect::<Result<Vec<_>, _>>()?;
        for table in &tables {
            match table.kind.as_str() {
                "controller" => {
                    controller(table.clone())?;
                }
                "input" => {
                    input(table.clone())?;
                }
                "output" | "driver" => {
                    output(table.clone())?;
                }
                "board" => {}
                "external_board" => {
                    external_board(table.clone())?;
                }
                "board_input" => {
                    board_input(table.clone())?;
                }
                "board_action" => {
                    board_action(table.clone())?;
                }
                "bus" => {
                    bus(table.clone())?;
                }
                "packet" => {
                    let spec = packet(table.clone())?;
                    let expected = match table.clone().text("kind")?.as_str() {
                        "wifi" => Some(sim_boards::EspRadioPort::Wifi),
                        "ble" => Some(sim_boards::EspRadioPort::Ble),
                        "esp_now" => Some(sim_boards::EspRadioPort::EspNow),
                        _ => None,
                    };
                    for (name, port) in [
                        (spec.from.as_deref(), spec.from_port),
                        (spec.to.as_deref(), spec.to_port),
                    ] {
                        let c3 = boards.iter().any(|board| {
                            Some(board.name.as_str()) == name
                                && board.catalog == sim_boards::BoardId::Esp32C3.key()
                                && board.catalog_asset.is_none()
                        });
                        if let (Some(_), Some(expected)) = (name, expected) {
                            let legacy_port =
                                port == 0 && !c3 && expected != sim_boards::EspRadioPort::EspNow;
                            let c3_port = port == expected as u32
                                && spec.profile.is_none_or(|profile| {
                                    profile.max_bytes <= expected.max_bytes()
                                });
                            if !(legacy_port || c3_port) {
                                return Err(table.error(
                                    "пакетный канал не соответствует порту или пределу C3",
                                ));
                            }
                        }
                    }
                }
                "transmission" => {
                    transmission(table.clone())?;
                }
                "fault" => {
                    fault(table.clone())?;
                }
                "stimulus" => {
                    stimulus(table.clone())?;
                }
                _ => return Err(table.error("неизвестная таблица")),
            }
        }
        Ok(Self {
            text: text.into(),
            tables,
        })
    }

    pub fn append_faults(&mut self, text: &str) -> Result<(), String> {
        let mut parsed = tables(text)?;
        parsed.remove(0).finish()?;
        for table in &parsed {
            if table.kind != "fault" {
                return Err(table.error("ожидается [[fault]]"));
            }
            fault(table.clone())?;
        }
        self.text.push('\n');
        self.text.push_str(text);
        self.tables.extend(parsed);
        Ok(())
    }

    /// Пересечение нового отказа до конца прогона с отказами в графе.
    pub fn fault_overlaps(&self, target: &str, from: Time) -> Result<bool, String> {
        for table in self.get("fault") {
            let existing = fault(table)?;
            if existing.target == target && existing.end.is_none_or(|end| end > from) {
                return Ok(true);
            }
        }
        Ok(false)
    }

    /// Файлы, читаемые при установке разрешённого графа.
    pub fn assets(&self) -> Result<BTreeSet<String>, String> {
        let mut names = BTreeSet::new();
        for item in self.get("board").map(board) {
            let item = item?;
            names.insert(item.image);
            names.extend(item.config);
            names.extend(item.catalog_asset);
            names.extend(item.flash_profile);
            names.extend(item.rom);
            names.extend(item.efuse);
            names.extend(item.power_profile);
            names.extend(item.power_schedule);
        }
        for item in self.get("external_board").map(external_board) {
            let item = item?;
            names.insert(item.profile);
            names.extend(item.image);
            names.extend(item.efuse);
        }
        for item in self.get("bus").map(bus) {
            names.insert(item?.model);
        }
        for item in self.get("packet").map(packet) {
            names.extend(item?.slots_asset);
        }
        for item in self.get("controller").map(controller) {
            names.insert(item?.program);
        }
        Ok(names)
    }

    /// Каталожные точки тока из отдельного документа `*-power.toml`, формат 1.
    pub fn parse_mcu_power_profile(text: &str) -> Result<McuPowerProfile, String> {
        let (mut header, states) = profile_tables(text)?;
        let board = header.text("board")?;
        let datasheet = header.text("datasheet")?;
        header.finish()?;
        let points = states
            .into_iter()
            .map(|mut t| {
                if t.kind != "states" {
                    return Err(t.error("ожидается [[states]]"));
                }
                let id = t.text("id")?;
                let current_a = t.finite("current_a")?;
                let kind = match t.text("kind")?.as_str() {
                    "typical" => McuCurrentKind::Typical,
                    "peak" => McuCurrentKind::Peak,
                    _ => return Err(t.error("неизвестный вид точки тока МК")),
                };
                let section = t.text("section")?;
                let table = t.text("table")?;
                let conditions = t.text("conditions")?;
                t.finish()?;
                Ok((
                    id,
                    McuCurrent {
                        current_a,
                        kind,
                        source: Provenance::Datasheet(format!(
                            "{datasheet}, §{section}, табл. {table}"
                        )),
                        conditions,
                    },
                ))
            })
            .collect::<Result<Vec<_>, _>>()?;
        if points.is_empty() {
            return Err("профиль потребления не содержит точек тока МК".into());
        }
        McuPowerProfile::new(board, points).map_err(|e| e.to_string())
    }

    /// Базовые задержки маршрутов с фиксированным временем доставки.
    pub fn wired_delays(&self) -> Result<BTreeMap<String, u64>, String> {
        let mut delays = BTreeMap::new();
        for item in self.get("packet").map(packet) {
            let item = item?;
            if item.profile.is_none() && item.slots_asset.is_none() && item.scheduled_rssi.is_none()
            {
                let name = format!("channel:{}", item.name);
                if delays.insert(name.clone(), item.delay_ns).is_some() {
                    return Err(format!("повтор провода «{name}»"));
                }
            }
        }
        Ok(delays)
    }

    pub(super) fn get<'a>(&'a self, kind: &'a str) -> impl Iterator<Item = Table> + 'a {
        self.tables
            .iter()
            .filter(move |table| table.kind == kind)
            .cloned()
    }

    pub(super) fn paired_setup(&self) -> u64 {
        let mut configuration = Vec::new();
        for table in &self.tables {
            let mut fields = table.fields.clone();
            if table.kind == "stimulus" {
                fields.remove("value");
            }
            configuration.push((&table.kind, fields));
        }
        sim_signal::fnv1a(format!("{configuration:?}").bytes())
    }
}

#[derive(Clone, Debug)]
pub(super) struct Table {
    pub kind: String,
    line: usize,
    fields: BTreeMap<String, String>,
}

impl Table {
    pub fn error(&self, detail: &str) -> String {
        format!("{} (строка {}): {detail}", self.kind, self.line)
    }

    fn take(&mut self, key: &str) -> Result<String, String> {
        self.fields
            .remove(key)
            .ok_or_else(|| self.error(&format!("нет поля «{key}»")))
    }

    pub fn has(&self, key: &str) -> bool {
        self.fields.contains_key(key)
    }

    pub fn text(&mut self, key: &str) -> Result<String, String> {
        let raw = self.take(key)?;
        let value = quoted(&raw).ok_or_else(|| self.error(&format!("ожидается строка «{key}»")))?;
        if value.trim().is_empty() {
            return Err(self.error(&format!("пустая строка «{key}»")));
        }
        Ok(value)
    }

    pub fn opt_text(&mut self, key: &str) -> Result<Option<String>, String> {
        self.has(key).then(|| self.text(key)).transpose()
    }

    pub fn number<T: FromStr>(&mut self, key: &str) -> Result<T, String> {
        let raw = self.take(key)?;
        parse_number(&raw).ok_or_else(|| self.error(&format!("неверное число «{key}»")))
    }

    fn opt_number<T: FromStr>(&mut self, key: &str) -> Result<Option<T>, String> {
        self.has(key).then(|| self.number(key)).transpose()
    }

    pub fn finite(&mut self, key: &str) -> Result<f64, String> {
        let value: f64 = self.number(key)?;
        if !value.is_finite() {
            return Err(self.error(&format!("неконечное число «{key}»")));
        }
        Ok(value)
    }

    pub fn opt_finite(&mut self, key: &str) -> Result<Option<f64>, String> {
        self.has(key).then(|| self.finite(key)).transpose()
    }

    pub fn boolean(&mut self, key: &str) -> Result<bool, String> {
        match self.take(key)?.as_str() {
            "true" => Ok(true),
            "false" => Ok(false),
            _ => Err(self.error(&format!("ожидается логическое значение «{key}»"))),
        }
    }

    pub fn array<T>(
        &mut self,
        key: &str,
        parse: impl Fn(&str) -> Option<T>,
    ) -> Result<Vec<T>, String> {
        let raw = self.take(key)?;
        let err = || self.error(&format!("некорректный массив «{key}»"));
        let inner = raw
            .strip_prefix('[')
            .and_then(|s| s.strip_suffix(']'))
            .ok_or_else(err)?
            .trim();
        let inner = inner.strip_suffix(',').unwrap_or(inner).trim();
        if inner.is_empty() {
            return Ok(Vec::new());
        }
        let mut values = Vec::new();
        let mut start = 0;
        scan(inner, |i, ch| {
            if ch == ',' {
                values.push(parse(inner[start..i].trim()));
                start = i + 1;
            }
            true
        });
        values.push(parse(inner[start..].trim()));
        values.into_iter().collect::<Option<_>>().ok_or_else(err)
    }

    pub fn bytes(&mut self, key: &str) -> Result<Vec<u8>, String> {
        self.array(key, parse_number)
    }

    pub fn rate(&mut self) -> Result<Rate, String> {
        let num = self.number("rate_hz")?;
        let den = if self.has("rate_den") {
            self.number("rate_den")?
        } else {
            1
        };
        Rate::ratio(num, den).ok_or_else(|| self.error("частота вне диапазона"))
    }

    pub fn source(&mut self) -> Result<String, String> {
        self.text("source")
    }

    pub fn finish(self) -> Result<(), String> {
        if let Some((key, _)) = self.fields.first_key_value() {
            Err(self.error(&format!("неизвестное поле «{key}»")))
        } else {
            Ok(())
        }
    }
}

pub(super) fn parse_number<T: FromStr>(raw: &str) -> Option<T> {
    if let Some(hex) = raw.strip_prefix("0x") {
        let digits = separated_digits(hex, |c| c.is_ascii_hexdigit())?;
        return u64::from_str_radix(&digits, 16)
            .ok()?
            .to_string()
            .parse()
            .ok();
    }
    if !raw
        .bytes()
        .next()
        .is_some_and(|b| b.is_ascii_digit() || matches!(b, b'+' | b'-'))
    {
        return None;
    }
    let value = separated_digits(raw, |c| c.is_ascii_digit())?;
    if value
        .bytes()
        .any(|b| !matches!(b, b'0'..=b'9' | b'+' | b'-' | b'.' | b'e' | b'E'))
    {
        return None;
    }
    let unsigned = value.trim_start_matches(['+', '-']);
    let integral = unsigned.split(['.', 'e', 'E']).next()?;
    if integral.is_empty()
        || integral.len() > 1 && integral.starts_with('0')
        || unsigned.contains(".e")
        || unsigned.contains(".E")
        || unsigned.ends_with('.')
    {
        return None;
    }
    value.parse().ok()
}

fn separated_digits(raw: &str, digit: impl Fn(u8) -> bool) -> Option<String> {
    let bytes = raw.as_bytes();
    for (i, b) in bytes.iter().enumerate() {
        if *b == b'_'
            && (i == 0 || !digit(bytes[i - 1]) || !bytes.get(i + 1).copied().is_some_and(&digit))
        {
            return None;
        }
    }
    Some(raw.replace('_', ""))
}

pub(super) fn quoted(raw: &str) -> Option<String> {
    if let Some(value) = raw.strip_prefix('\'').and_then(|s| s.strip_suffix('\'')) {
        return (!value.contains(['\'', '\n', '\r']) && !value.chars().any(char::is_control))
            .then(|| value.into());
    }
    let mut chars = raw.strip_prefix('"')?.strip_suffix('"')?.chars();
    let mut out = String::new();
    while let Some(ch) = chars.next() {
        out.push(match ch {
            '\\' => match chars.next()? {
                '"' => '"',
                '\\' => '\\',
                'n' => '\n',
                'r' => '\r',
                't' => '\t',
                'b' => '\x08',
                'f' => '\x0c',
                marker @ ('u' | 'U') => {
                    let count = if marker == 'u' { 4 } else { 8 };
                    let hex: String = chars.by_ref().take(count).collect();
                    if hex.len() != count || !hex.bytes().all(|b| b.is_ascii_hexdigit()) {
                        return None;
                    }
                    char::from_u32(u32::from_str_radix(&hex, 16).ok()?)?
                }
                _ => return None,
            },
            '"' => return None,
            ch if ch.is_control() => return None,
            ch => ch,
        });
    }
    Some(out)
}

fn scan(line: &str, mut unquoted: impl FnMut(usize, char) -> bool) {
    let mut quote = None;
    let mut escape = false;
    for (i, ch) in line.char_indices() {
        match quote {
            Some('"') if escape => escape = false,
            Some('"') if ch == '\\' => escape = true,
            Some(marker) if ch == marker => quote = None,
            Some(_) => (),
            None if matches!(ch, '\'' | '"') => quote = Some(ch),
            None if !unquoted(i, ch) => return,
            None => (),
        }
    }
}

/// Подмножество TOML для плоских таблиц и однострочных значений.
pub(super) fn tables(text: &str) -> Result<Vec<Table>, String> {
    let mut out = vec![Table {
        kind: "заголовок".into(),
        line: 1,
        fields: BTreeMap::new(),
    }];
    for (i, line) in text.lines().enumerate() {
        let mut end = line.len();
        scan(line, |j, ch| {
            if ch == '#' {
                end = j;
                return false;
            }
            true
        });
        let line = line[..end].trim();
        if line.is_empty() {
            continue;
        }
        let bare = |key: &str| {
            !key.is_empty()
                && key
                    .bytes()
                    .all(|b| b.is_ascii_alphanumeric() || matches!(b, b'_' | b'-'))
        };
        if let Some(kind) = line
            .strip_prefix("[[")
            .and_then(|s| s.strip_suffix("]]"))
            .map(str::trim)
            .filter(|s| bare(s))
        {
            out.push(Table {
                kind: kind.into(),
                line: i + 1,
                fields: BTreeMap::new(),
            });
        } else if let Some((key, value)) = line
            .split_once('=')
            .map(|(k, v)| (k.trim(), v.trim()))
            .filter(|(k, v)| bare(k) && !v.is_empty())
        {
            if out
                .last_mut()
                .expect("есть заголовок")
                .fields
                .insert(key.into(), value.into())
                .is_some()
            {
                return Err(format!("строка {}: повтор поля «{key}»", i + 1));
            }
        } else {
            return Err(format!("строка {}: неверная запись TOML", i + 1));
        }
    }
    Ok(out)
}

#[derive(Clone, Debug)]
pub(super) struct ControllerSpec {
    pub name: String,
    pub rate: Rate,
    pub program: String,
    pub external: Option<ExternalSpec>,
}

#[derive(Clone, Debug)]
pub(super) struct ExternalSpec {
    pub args: Vec<String>,
    pub incoming: Delay,
    pub outgoing: Delay,
    pub request_rate: Rate,
    pub request_max_age_ns: u64,
    pub queue_capacity: usize,
    pub max_message_bytes: u32,
    pub max_history_bytes: Option<usize>,
}

pub(super) fn controller(mut t: Table) -> Result<ControllerSpec, String> {
    let name = t.text("name")?;
    let rate = t.rate()?;
    let program = t.text("program")?;
    let external = match t.text("kind")?.as_str() {
        "builtin" => None,
        "external" => Some(ExternalSpec {
            args: t.array("args", quoted)?,
            incoming: delay(&mut t, "input")?,
            outgoing: delay(&mut t, "output")?,
            request_rate: Rate::hz(t.number("request_hz")?)
                .ok_or_else(|| t.error("неверная частота внешней стороны"))?,
            request_max_age_ns: t.number("request_max_age_ns")?,
            queue_capacity: t.number("queue_capacity")?,
            max_message_bytes: t.number("max_message_bytes")?,
            max_history_bytes: t
                .has("max_history_bytes")
                .then(|| t.number("max_history_bytes"))
                .transpose()?,
        }),
        _ => return Err(t.error("неизвестный вид контроллера")),
    };
    if external
        .as_ref()
        .is_some_and(|e| e.queue_capacity == 0 || e.max_message_bytes == 0)
    {
        return Err(t.error("нулевая ёмкость транспорта"));
    }
    t.source()?;
    t.finish()?;
    Ok(ControllerSpec {
        name,
        rate,
        program,
        external,
    })
}

fn delay(t: &mut Table, prefix: &str) -> Result<Delay, String> {
    let latency_ns: u64 = t.number(&format!("{prefix}_latency_ns"))?;
    let jitter_ns: u64 = t.number(&format!("{prefix}_jitter_ns"))?;
    if jitter_ns > latency_ns || latency_ns.checked_add(jitter_ns).is_none() {
        return Err(t.error("джиттер выходит за допустимый диапазон"));
    }
    Ok(Delay {
        latency_ns,
        jitter_ns,
    })
}

pub(super) struct InputSpec {
    pub controller: String,
    pub signal: String,
    pub unit: String,
    pub kind: InputKind,
}

pub(super) fn input(mut t: Table) -> Result<InputSpec, String> {
    let controller = t.text("controller")?;
    let signal = t.text("signal")?;
    let unit = t.text("unit")?;
    let kind = match t.text("kind")?.as_str() {
        "scalar" => InputKind::Scalar {
            max_age_ns: t.number("max_age_ns")?,
        },
        kind @ ("rgb8" | "depth_mm16") => InputKind::Image {
            format: if kind == "rgb8" {
                PixelFormat::Rgb8
            } else {
                PixelFormat::DepthMm16
            },
            max_capture_age_ns: t.number("max_capture_age_ns")?,
            max_delivery_age_ns: t.number("max_delivery_age_ns")?,
        },
        _ => return Err(t.error("неизвестный тип сигнала")),
    };
    t.source()?;
    t.finish()?;
    Ok(InputSpec {
        controller,
        signal,
        unit,
        kind,
    })
}

#[derive(Clone, Debug)]
pub(super) struct OutputSpec {
    pub name: String,
    pub owner: String,
    pub target: String,
    pub target_kind: TargetKind,
    pub unit: String,
    pub max_age_ns: u64,
    pub fallback: Option<f64>,
    /// `fallback = "hold"`: при пропуске держать последнее значение, ушедшее на выход.
    pub hold_last: bool,
    pub ack: Option<String>,
    pub driver: Option<DriverSpec>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum TargetKind {
    Goal,
    Effort,
    Drive,
    Power,
}

#[derive(Clone, Debug)]
pub(super) struct DriverSpec {
    pub pin: u32,
    pub quantity: String,
    pub direction_pin: Option<u32>,
    pub enable_pin: Option<u32>,
    pub active_high: Option<bool>,
    pub period_min_s: Option<f64>,
    pub period_max_s: Option<f64>,
    pub scale: f64,
    pub offset: f64,
}

pub(super) fn output(mut t: Table) -> Result<OutputSpec, String> {
    let owner = if t.kind == "driver" {
        "board"
    } else {
        "controller"
    };
    let driver = (owner == "board")
        .then(|| {
            let quantity = t.text("quantity")?;
            if !matches!(quantity.as_str(), "logic" | "duty" | "high_s" | "period_s") {
                return Err(t.error("неизвестная величина драйвера"));
            }
            let pin = t.number("pin")?;
            let direction_pin = t
                .has("direction_pin")
                .then(|| t.number("direction_pin"))
                .transpose()?;
            let enable_pin = t
                .has("enable_pin")
                .then(|| t.number("enable_pin"))
                .transpose()?;
            let active_high = enable_pin
                .map(|_| t.boolean("enable_active_high"))
                .transpose()?;
            let period_min_s = t.opt_finite("period_min_s")?;
            let period_max_s = t.opt_finite("period_max_s")?;
            match (period_min_s, period_max_s) {
                (Some(min), Some(max)) if quantity != "logic" && min > 0.0 && max >= min => (),
                (None, None) if !matches!(quantity.as_str(), "high_s" | "period_s") => (),
                _ => return Err(t.error("неверный диапазон периода драйвера")),
            }
            Ok(DriverSpec {
                pin,
                quantity,
                direction_pin,
                enable_pin,
                active_high,
                period_min_s,
                period_max_s,
                scale: t.finite("scale")?,
                offset: t.finite("offset")?,
            })
        })
        .transpose()?;
    let name = t.text("name")?;
    let owner = t.text(owner)?;
    let target = t.text("target")?;
    let target_kind = match t.text("target_kind")?.as_str() {
        "goal" => TargetKind::Goal,
        "effort" => TargetKind::Effort,
        "drive" => TargetKind::Drive,
        "power" => TargetKind::Power,
        _ => return Err(t.error("неизвестный тип получателя")),
    };
    let unit = t.text("unit")?;
    let max_age_ns = t.number("max_age_ns")?;
    // Число — подстановка значения, строка "hold" — последнее значение выхода.
    let hold_last = t
        .fields
        .get("fallback")
        .is_some_and(|raw| quoted(raw).as_deref() == Some("hold"));
    let fallback = if hold_last {
        t.text("fallback")?;
        None
    } else {
        t.opt_finite("fallback")?
    };
    let ack = t.opt_text("ack")?;
    if (target_kind == TargetKind::Drive) != ack.is_some() {
        return Err(t.error("для входа привода требуется канал подтверждения"));
    }
    if target_kind == TargetKind::Power && fallback.is_some_and(|v| v != 0.0 && v != 1.0) {
        return Err(t.error("резерв питания должен быть 0 или 1"));
    }
    if matches!(target_kind, TargetKind::Drive) && fallback.is_none() && !hold_last {
        return Err(t.error("для привода нужна явная реакция на пропуск команды"));
    }
    if hold_last && !matches!(target_kind, TargetKind::Drive) {
        return Err(t.error("резерв «hold» допустим только для привода"));
    }
    t.source()?;
    t.finish()?;
    Ok(OutputSpec {
        name,
        owner,
        target,
        target_kind,
        unit,
        max_age_ns,
        fallback,
        hold_last,
        ack,
        driver,
    })
}

pub(super) struct BoardSpec {
    pub name: String,
    pub catalog: String,
    pub image: String,
    pub config: Option<String>,
    pub catalog_asset: Option<String>,
    pub flash_profile: Option<String>,
    pub rom: Option<String>,
    pub efuse: Option<String>,
    pub power_profile: Option<String>,
    pub power_schedule: Option<String>,
    pub rate: Rate,
    pub end_sample: bool,
}

pub(super) fn board(mut t: Table) -> Result<BoardSpec, String> {
    let item = BoardSpec {
        name: t.text("name")?,
        catalog: t.text("catalog")?,
        image: t.text("image")?,
        config: t.opt_text("config")?,
        catalog_asset: t.opt_text("catalog_asset")?,
        flash_profile: t.opt_text("flash_profile")?,
        rom: t.opt_text("rom")?,
        efuse: t.opt_text("efuse")?,
        power_profile: t.opt_text("power_profile")?,
        power_schedule: t.opt_text("power_schedule")?,
        rate: t.rate()?,
        end_sample: if t.has("end_sample") {
            t.boolean("end_sample")?
        } else {
            false
        },
    };
    if (item.rom.is_some(), item.efuse.is_some())
        != (item.flash_profile.is_some(), item.flash_profile.is_some())
    {
        return Err(t.error("flash_profile, rom и efuse задаются вместе"));
    }
    if item.power_schedule.is_some() && item.power_profile.is_none() {
        return Err(t.error("power_schedule требует power_profile"));
    }
    t.source()?;
    t.finish()?;
    Ok(item)
}

fn profile_tables(text: &str) -> Result<(Table, Vec<Table>), String> {
    let mut tables = tables(text)?;
    let mut header = tables.remove(0);
    if header.number::<u32>("format")? != 1 {
        return Err(header.error("неподдержанная версия профиля платы"));
    }
    Ok((header, tables))
}

fn profile_header(text: &str) -> Result<Table, String> {
    let (header, tables) = profile_tables(text)?;
    if let Some(table) = tables.first() {
        return Err(table.error("лишняя таблица в профиле платы"));
    }
    Ok(header)
}

pub(super) struct FlashProfile {
    pub board: String,
    pub partition_table_offset: u32,
    pub straps: [bool; 3],
    pub options: sim_boards::EspBootOptions,
}

pub(super) fn flash_profile(text: &str) -> Result<FlashProfile, String> {
    use sim_boards::{EspBootOptions, SystimerPhase, esp_flash::EspFlashReadback};

    let mut t = profile_header(text)?;
    let board = t.text("board")?;
    // ESP-IDF components/partition_table/Kconfig.projbuild: CONFIG_PARTITION_TABLE_OFFSET.
    let partition_table_offset = t.number("partition_table_offset")?;
    // ESP32-C3 TRM §5.8 GPIO_STRAP_REG. Порядок входов задаёт BoardSim::load_esp_flash.
    let straps = [
        t.boolean("gpio2")?,
        t.boolean("gpio8")?,
        t.boolean("gpio9")?,
    ];
    // ESP32-C3 TRM §10.3: счётчик использует два делителя тактов XTAL.
    let systimer_phase = match t.opt_number::<u8>("systimer_first_xtal_cycles")? {
        None => None,
        Some(2) => Some(SystimerPhase::FirstAfterTwoXtalCycles),
        Some(3) => Some(SystimerPhase::FirstAfterThreeXtalCycles),
        Some(_) => return Err(t.error("неверная начальная фаза SYSTIMER")),
    };
    // ESP-IDF components/spi_flash/{spi_flash_chip_generic,memspi_host_driver}.c:
    // RDSR/RDSR2 и RDID. Значения и порядок байтов берутся из паспорта выбранной flash.
    // Значения берутся из профиля конкретной flash, в порядке байтов на шине.
    let flash_readback = if t.has("flash_jedec_id") {
        Some(EspFlashReadback {
            jedec_id: t
                .bytes("flash_jedec_id")?
                .try_into()
                .map_err(|_| t.error("RDID ожидает три байта JEDEC"))?,
            status1: t.number("flash_status1")?,
            status2: t.opt_number("flash_status2")?,
        })
    } else {
        None
    };
    t.source()?;
    t.finish()?;
    Ok(FlashProfile {
        board,
        partition_table_offset,
        straps,
        options: EspBootOptions {
            systimer_phase,
            flash_readback,
        },
    })
}

pub(super) struct PowerSchedule {
    pub states: Vec<(String, Time)>,
    pub repeat: bool,
    pub source: Provenance,
}

pub(super) fn power_schedule(text: &str) -> Result<PowerSchedule, String> {
    let (mut header, rows) = profile_tables(text)?;
    let repeat = header.boolean("repeat")?;
    let source = Provenance::Estimate(header.source()?);
    header.finish()?;
    let states = rows
        .into_iter()
        .map(|mut t| {
            if t.kind != "states" {
                return Err(t.error("ожидается [[states]]"));
            }
            let id = t.text("id")?;
            let duration = Time(t.number("duration_ns")?);
            if duration == Time::ZERO {
                return Err(t.error("пустой интервал состояния МК"));
            }
            t.finish()?;
            Ok((id, duration))
        })
        .collect::<Result<Vec<_>, _>>()?;
    if states.is_empty() {
        return Err("график потребления не содержит состояний МК".into());
    }
    Ok(PowerSchedule {
        states,
        repeat,
        source,
    })
}

pub(super) struct ExternalBoardSpec {
    pub name: String,
    pub catalog: String,
    pub profile: String,
    pub image: Option<String>,
    pub efuse: Option<String>,
}

pub(super) fn external_board(mut t: Table) -> Result<ExternalBoardSpec, String> {
    let item = ExternalBoardSpec {
        name: t.text("name")?,
        catalog: t.text("catalog")?,
        profile: t.text("profile")?,
        image: t.opt_text("image")?,
        efuse: t.opt_text("efuse")?,
    };
    t.source()?;
    t.finish()?;
    Ok(item)
}

pub(super) struct ExternalProfile {
    pub board: String,
    pub qemu: String,
    pub flash_boot: bool,
    pub psram: Option<super::external_board::Psram>,
    pub clock: super::external_board::GuestClock,
    pub seed: u64,
    pub limits: super::external_board::Limits,
    pub ports: Vec<String>,
}

pub(super) fn external_profile(text: &str) -> Result<ExternalProfile, String> {
    use super::external_board::{
        BlockCoverage, EXTERNAL_BOARD_PORTS, GuestClock, Limits, Psram, esp32s3_coverage,
    };

    let mut t = profile_header(text)?;
    let board = t.text("board")?;
    let qemu = t.text("qemu")?;
    // ESP-IDF, руководство ESP32-S3 «Run QEMU», машина esp32s3.
    if t.text("machine")? != "esp32s3" {
        return Err(t.error("несовместимая машина внешней платы"));
    }
    let flash_boot = match t.text("boot")?.as_str() {
        "flash" => true,
        "uart_download" => false,
        _ => return Err(t.error("неизвестный режим загрузки QEMU")),
    };
    let psram = if let Some(mib) = t.opt_number("psram_mib")? {
        // QEMU Espressif для S3, «Adding PSRAM».
        if ![2, 4, 8, 16, 32].contains(&mib) {
            return Err(t.error("неподдержанный объём PSRAM"));
        }
        Some(Psram {
            mib,
            octal: t.boolean("psram_octal")?,
        })
    } else {
        None
    };
    let clock = match t.text("clock")?.as_str() {
        "icount" => {
            let shift = t.number("icount_shift")?;
            // QEMU Espressif, accel/tcg/icount-common.c, MAX_ICOUNT_SHIFT.
            if shift > 10 {
                return Err(t.error("icount_shift вне диапазона 0–10"));
            }
            GuestClock::InstructionCount { shift }
        }
        "diagnostic" => GuestClock::Diagnostic,
        _ => return Err(t.error("неизвестные часы гостя QEMU")),
    };
    let seed = t.number("seed")?;
    let limits = Limits {
        wall_time: std::time::Duration::from_millis(t.number("wall_time_ms")?),
        address_space_bytes: t.number("address_space_bytes")?,
        uart_bytes: t.number("uart_bytes")?,
        stderr_bytes: t.number("stderr_bytes")?,
    };
    // sys/resource.h Darwin: RLIM_INFINITY = INT64_MAX.
    if limits.wall_time.is_zero()
        || limits.address_space_bytes == 0
        || limits.address_space_bytes >= i64::MAX as u64
        || limits.uart_bytes == 0
        || limits.stderr_bytes == 0
    {
        return Err(t.error("нужны положительные конечные лимиты QEMU"));
    }
    let blocks = t.array("required_blocks", quoted)?;
    let ports = t.array("required_ports", quoted)?;
    if blocks.iter().collect::<BTreeSet<_>>().len() != blocks.len()
        || ports.iter().collect::<BTreeSet<_>>().len() != ports.len()
    {
        return Err(t.error("повтор блока или порта QEMU"));
    }
    for block in &blocks {
        if let BlockCoverage::NotAnalyzed { reason } = esp32s3_coverage(block) {
            return Err(t.error(&format!("несовместимый блок «{block}»: {reason}")));
        }
        let compatible = match block.as_str() {
            "psram_qpi" => psram.is_some_and(|p| !p.octal),
            "psram_opi" => psram.is_some_and(|p| p.octal),
            "psram_mmu" => psram.is_some(),
            _ => true,
        };
        if !compatible {
            return Err(t.error(&format!("блок «{block}» несовместим с настройками PSRAM")));
        }
    }
    for port in &ports {
        if !EXTERNAL_BOARD_PORTS.contains(&port.as_str()) {
            return Err(t.error(&format!("порт «{port}» не подключён адаптером QEMU")));
        }
    }
    t.source()?;
    t.finish()?;
    Ok(ExternalProfile {
        board,
        qemu,
        flash_boot,
        psram,
        clock,
        seed,
        limits,
        ports,
    })
}

pub(super) struct BoardWire {
    pub name: String,
    pub board: String,
    pub signal: String,
    pub unit: String,
    pub kind: String,
    pub port: u32,
    pub max_age_ns: u64,
    pub scale: f64,
    pub offset: f64,
    pub fallback: Option<f64>,
}

pub(super) fn board_input(mut t: Table) -> Result<BoardWire, String> {
    let item = BoardWire {
        name: t.text("name")?,
        board: t.text("board")?,
        signal: t.text("signal")?,
        unit: t.text("unit")?,
        kind: t.text("kind")?,
        port: t.number("port")?,
        max_age_ns: t.number("max_age_ns")?,
        scale: t.finite("scale")?,
        offset: t.finite("offset")?,
        fallback: t.opt_finite("fallback")?,
    };
    if !matches!(item.kind.as_str(), "adc" | "gpio")
        || item.kind == "gpio" && !item.fallback.is_some_and(|v| v == 0.0 || v == 1.0)
    {
        return Err(t.error("неизвестный вход платы или отсутствует логический уровень обрыва"));
    }
    t.source()?;
    t.finish()?;
    Ok(item)
}

pub(super) enum BoardActionSpec {
    Gpio {
        pin: u32,
        high: bool,
    },
    Reset {
        config: Option<String>,
        restart_clock: bool,
    },
}

pub(super) fn board_action(
    mut t: Table,
) -> Result<(String, Time, String, BoardActionSpec), String> {
    let id = t.text("id")?;
    let at = Time(t.number("at_ns")?);
    let board = t.text("board")?;
    let kind = t.text("kind")?;
    let action = match kind.as_str() {
        "gpio" => BoardActionSpec::Gpio {
            pin: t.number("pin")?,
            high: t.boolean("high")?,
        },
        "reset" | "restart" => BoardActionSpec::Reset {
            config: t.opt_text("config")?,
            restart_clock: kind == "restart",
        },
        _ => return Err(t.error("неизвестное действие платы")),
    };
    t.source()?;
    t.finish()?;
    Ok((id, at, board, action))
}

pub(super) struct BusSpec {
    pub name: String,
    pub board: String,
    pub bus: u32,
    pub signal: String,
    pub model: String,
    pub chip_select: Option<u8>,
    pub max_age_ns: u64,
}

pub(super) fn bus(mut t: Table) -> Result<BusSpec, String> {
    let item = BusSpec {
        name: t.text("name")?,
        board: t.text("board")?,
        bus: t.number("bus")?,
        signal: t.text("signal")?,
        model: t.text("model")?,
        chip_select: t
            .has("chip_select")
            .then(|| t.number("chip_select"))
            .transpose()?,
        max_age_ns: t.number("max_age_ns")?,
    };
    t.source()?;
    t.finish()?;
    Ok(item)
}

pub(super) struct PacketSpec {
    pub name: String,
    pub from: Option<String>,
    pub from_port: u32,
    pub to: Option<String>,
    pub to_port: u32,
    pub profile: Option<sim_radio::timed::Profile>,
    pub scheduled_rssi: Option<i8>,
    pub scheduled_max_bytes: Option<usize>,
    pub slots_asset: Option<String>,
    pub delay_ns: u64,
    pub max_age_ns: u64,
    pub capacity: usize,
}

pub(super) fn packet(mut t: Table) -> Result<PacketSpec, String> {
    use sim_radio::timed::{Profile, Protocol};
    let kind = t.text("kind")?;
    let profile = match kind.as_str() {
        "uart" | "scheduled_radio" | "esp_now" => None,
        "ble" | "wifi" => Some(Profile {
            protocol: if kind == "ble" {
                Protocol::Ble {
                    interval_ns: t.number("interval_ns")?,
                }
            } else {
                Protocol::Wifi {
                    attempt_ns: t.number("attempt_ns")?,
                }
            },
            loss: t.finite("loss")?,
            rssi_dbm: t.number("rssi_dbm")?,
            attempts: t.number("attempts")?,
            capacity: t.number("capacity")?,
            max_bytes: t.number("max_bytes")?,
        }),
        _ => return Err(t.error("неизвестный пакетный канал")),
    };
    let scheduled_rssi = if kind == "scheduled_radio" {
        let rssi = t.number("rssi_dbm")?;
        if !(-127..=0).contains(&rssi) {
            return Err(t.error("RSSI вне диапазона −127…0 дБм"));
        }
        Some(rssi)
    } else {
        None
    };
    let scheduled_max_bytes = (scheduled_rssi.is_some() || kind == "esp_now")
        .then(|| t.number("max_bytes"))
        .transpose()?;
    if scheduled_max_bytes == Some(0) {
        return Err(t.error("пакетный канал требует положительный предел размера"));
    }
    let name = t.text("name")?;
    let from = t.opt_text("from_board")?;
    let from_port = if from.is_some() {
        t.number("from_port")?
    } else {
        0
    };
    let to = t.opt_text("to_board")?;
    let to_port = if to.is_some() {
        t.number("to_port")?
    } else {
        0
    };
    let capacity = if let Some(profile) = profile {
        profile.capacity
    } else {
        t.number("capacity")?
    };
    let max_age_ns = t.number("max_age_ns")?;
    let slots_asset = match kind.as_str() {
        "scheduled_radio" => t.opt_text("slots_asset")?,
        "esp_now" => Some(t.text("profile_asset")?),
        _ => None,
    };
    // Календарь задаёт моменты доставки. Постоянная задержка задаёт другой режим канала.
    let delay_ns = if profile.is_none() && slots_asset.is_none() {
        t.number("delay_ns")?
    } else {
        0
    };
    // Внешний приёмник принимает пакеты через журнал доставки штатного канала.
    let to_host = t.has("to_host") && t.boolean("to_host")?;
    if capacity == 0
        || (to_host && (to.is_some() || (profile.is_none() && kind != "esp_now")))
        || (from.is_none() && to.is_none() && !to_host)
    {
        return Err(t.error("у канала нет ни одного подключённого порта"));
    }
    t.source()?;
    t.finish()?;
    Ok(PacketSpec {
        name,
        from,
        from_port,
        to,
        to_port,
        profile,
        scheduled_rssi,
        scheduled_max_bytes,
        slots_asset,
        delay_ns,
        max_age_ns,
        capacity,
    })
}

#[derive(Clone, Copy)]
pub(super) struct DeliverySlot {
    pub accepts_through: Time,
    pub delivers_at: Time,
}

#[derive(Clone)]
pub(super) struct EspNowPacketProfile {
    pub sender: EspNowProfile,
    pub peer: EspNowPeer,
    pub link: EspNowLink,
    pub confirmation: EspNowConfirmation,
    pub reply: Option<sim_radio::timed::Profile>,
}

fn esp_now_version(t: &Table, version: u32) -> Result<EspNowVersion, String> {
    match version {
        // ESP-IDF v6.1.1, esp_now.rst, Frame Format.
        1 => Ok(EspNowVersion::V1),
        2 => Ok(EspNowVersion::V2),
        _ => Err(t.error("версия ESP-NOW вне диапазона")),
    }
}

fn esp_now_packet_profile(text: &str) -> Result<EspNowPacketProfile, String> {
    let mut t = profile_header(text)?;
    if t.text("kind")? != "esp_now" {
        return Err(t.error("ожидается профиль ESP-NOW"));
    }
    let version = t.number("sender_version")?;
    let sender = EspNowProfile {
        version: esp_now_version(&t, version)?,
        wifi_channel: t.number("wifi_channel")?,
        station_enabled: t.boolean("station_enabled")?,
        soft_ap_enabled: t.boolean("soft_ap_enabled")?,
        queue_capacity: Some(t.number("queue_capacity")?),
        attempt_ns: Some(t.number("attempt_ns")?),
        mac_attempts: t.opt_number("mac_attempts")?,
    };
    if sender.queue_capacity == Some(0)
        || sender.attempt_ns == Some(0)
        || sender.mac_attempts == Some(0)
    {
        return Err(t.error("очередь, длительность и лимит попыток должны быть положительными"));
    }
    let address = t
        .bytes("destination")?
        .try_into()
        .map_err(|_| t.error("ожидается MAC-адрес из шести октетов"))?;
    let peer = EspNowPeer {
        address,
        interface: match t.text("interface")?.as_str() {
            "station" => EspNowInterface::Station,
            "soft_ap" => EspNowInterface::SoftAp,
            _ => return Err(t.error("неизвестный интерфейс ESP-NOW")),
        },
        channel: t.number("peer_channel")?,
        encrypted: t.boolean("encrypted")?,
    };
    let version = t.opt_number("receiver_version")?;
    let link = EspNowLink {
        frame: AttemptLink {
            loss: t.finite("loss")?,
            rssi_dbm: t.number("rssi_dbm")?,
        },
        receiver_channel: t.opt_number("receiver_channel")?,
        receiver_version: version.map(|v| esp_now_version(&t, v)).transpose()?,
        mac_ack_loss: t.opt_finite("mac_ack_loss")?,
    };
    // ESP-IDF, Send ESP-NOW Data: ответ приложения передаётся отдельно от MAC ACK.
    let confirmation = match t.text("confirmation")?.as_str() {
        "driver" => EspNowConfirmation::Driver,
        "application" => EspNowConfirmation::Application {
            timeout_ns: Some(t.number("timeout_ns")?),
            transmissions: Some(t.number("transmissions")?),
        },
        _ => return Err(t.error("неизвестный тип подтверждения ESP-NOW")),
    };
    let reply = if matches!(confirmation, EspNowConfirmation::Application { .. })
        && t.has("reply_attempt_ns")
    {
        let attempt_ns = t.number("reply_attempt_ns")?;
        let loss = t.finite("reply_loss")?;
        if attempt_ns == 0 || !(0.0..=1.0).contains(&loss) {
            return Err(t.error("некорректные длительность и потери ответа приложения"));
        }
        Some(sim_radio::timed::Profile {
            protocol: sim_radio::timed::Protocol::Wifi { attempt_ns },
            loss,
            rssi_dbm: t.number("reply_rssi_dbm")?,
            // Один прикладной ответ на вызов ACK. Нагрузка адаптера
            // содержит сквозной u64; реальный формат задаёт приложение.
            attempts: 1,
            capacity: sender.queue_capacity.expect("обязательное поле профиля"),
            max_bytes: size_of::<u64>(),
        })
    } else {
        None
    };
    if !(0.0..=1.0).contains(&link.frame.loss)
        || link.mac_ack_loss.is_some_and(|v| !(0.0..=1.0).contains(&v))
    {
        return Err(t.error("вероятность потери вне диапазона 0…1"));
    }
    t.source()?;
    t.finish()?;
    Ok(EspNowPacketProfile {
        sender,
        peer,
        link,
        confirmation,
        reply,
    })
}

/// Для каждого слота заданы последние допустимые наносекунды готовности и
/// момент доставки. Обе колонки строго возрастают, готовность включительна.
pub(super) fn delivery_slots(bytes: &[u8]) -> Result<super::board::PacketPlan, String> {
    let text = std::str::from_utf8(bytes).map_err(|e| e.to_string())?;
    let mut lines = text.lines();
    if lines.next() != Some("accepts_through_ns,delivers_at_ns") {
        return esp_now_packet_profile(text).map(super::board::PacketPlan::esp_now);
    }
    let mut slots = Vec::<DeliverySlot>::new();
    for (i, line) in lines.enumerate() {
        let (accepts_through, delivers_at) = line
            .split_once(',')
            .and_then(|(a, b)| Some((Time(parse_number(a)?), Time(parse_number(b)?))))
            .ok_or_else(|| format!("неверное время слота доставки в строке {}", i + 2))?;
        if accepts_through > delivers_at
            || slots.last().is_some_and(|last| {
                last.accepts_through >= accepts_through || last.delivers_at >= delivers_at
            })
        {
            return Err(format!(
                "нарушен порядок слотов доставки в строке {}",
                i + 2
            ));
        }
        slots.push(DeliverySlot {
            accepts_through,
            delivers_at,
        });
    }
    if slots.is_empty() {
        return Err("пустой календарь доставки".into());
    }
    Ok(super::board::PacketPlan::Calendar(slots.into()))
}

pub(super) fn transmission(mut t: Table) -> Result<(Time, String, Vec<u8>), String> {
    let item = (
        Time(t.number("at_ns")?),
        t.text("route")?,
        t.bytes("bytes")?,
    );
    t.source()?;
    t.finish()?;
    Ok(item)
}

pub(super) fn stimulus(mut t: Table) -> Result<(Time, String, String, f64), String> {
    let item = (
        Time(t.number("at_ns")?),
        t.text("target")?,
        t.text("unit")?,
        t.finite("value")?,
    );
    t.source()?;
    t.finish()?;
    Ok(item)
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FaultAction {
    Drop,
    Stuck(f64),
    PowerOff,
    Pause,
    Delay(u64),
}

#[derive(Clone, Debug)]
pub(super) struct FaultSpec {
    pub id: String,
    pub target: String,
    pub start: Time,
    pub end: Option<Time>,
    pub action: FaultAction,
}

pub(super) fn fault(mut t: Table) -> Result<FaultSpec, String> {
    let id = t.text("id")?;
    let target = t.text("target")?;
    let start = Time(t.number("at_ns")?);
    let end = t
        .has("until_ns")
        .then(|| t.number("until_ns").map(Time))
        .transpose()?;
    if end.is_some_and(|end| end <= start) {
        return Err(t.error("конец отказа должен следовать за началом"));
    }
    let action = match t.text("kind")?.as_str() {
        "drop" => FaultAction::Drop,
        "stuck" => FaultAction::Stuck(t.finite("value")?),
        "power_off" => FaultAction::PowerOff,
        "pause" => FaultAction::Pause,
        "delay" => {
            let ns = t.number("delay_ns")?;
            if ns == 0 {
                return Err(t.error("добавочная задержка должна быть положительной"));
            }
            FaultAction::Delay(ns)
        }
        _ => return Err(t.error("неподдержанный механизм отказа")),
    };
    t.source()?;
    t.finish()?;
    Ok(FaultSpec {
        id,
        target,
        start,
        end,
        action,
    })
}
