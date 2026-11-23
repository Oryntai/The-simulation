use std::collections::{BTreeMap, BTreeSet};
use std::str::FromStr;

use sim_control::{Delay, InputKind};
use sim_signal::{PixelFormat, Rate, Time};

/// Документ версии 1. Неизвестные, повторные и неприменимые поля отклоняются.
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
                "board" => {
                    board(table.clone())?;
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
                    packet(table.clone())?;
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
    let fallback = t.opt_finite("fallback")?;
    let ack = t.opt_text("ack")?;
    if (target_kind == TargetKind::Drive) != ack.is_some() {
        return Err(t.error("для входа привода требуется канал подтверждения"));
    }
    if target_kind == TargetKind::Power && fallback.is_some_and(|v| v != 0.0 && v != 1.0) {
        return Err(t.error("резерв питания должен быть 0 или 1"));
    }
    if matches!(target_kind, TargetKind::Drive) && fallback.is_none() {
        return Err(t.error("для привода нужна явная реакция на пропуск команды"));
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
        ack,
        driver,
    })
}

pub(super) struct BoardSpec {
    pub name: String,
    pub catalog: String,
    pub image: String,
    pub config: Option<String>,
    pub rate: Rate,
    pub end_sample: bool,
}

pub(super) fn board(mut t: Table) -> Result<BoardSpec, String> {
    let item = BoardSpec {
        name: t.text("name")?,
        catalog: t.text("catalog")?,
        image: t.text("image")?,
        config: t.opt_text("config")?,
        rate: t.rate()?,
        end_sample: if t.has("end_sample") {
            t.boolean("end_sample")?
        } else {
            false
        },
    };
    t.source()?;
    t.finish()?;
    Ok(item)
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
        "uart" | "scheduled_radio" => None,
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
    let scheduled_max_bytes = scheduled_rssi.map(|_| t.number("max_bytes")).transpose()?;
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
    let slots_asset = if kind == "scheduled_radio" {
        t.opt_text("slots_asset")?
    } else {
        None
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
        || (to_host && (to.is_some() || profile.is_none()))
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

/// Для каждого слота заданы последние допустимые наносекунды готовности и
/// момент доставки. Обе колонки строго возрастают, готовность включительна.
pub(super) fn delivery_slots(bytes: &[u8]) -> Result<Vec<DeliverySlot>, String> {
    let text = std::str::from_utf8(bytes).map_err(|e| e.to_string())?;
    let mut lines = text.lines();
    if lines.next() != Some("accepts_through_ns,delivers_at_ns") {
        return Err("неверный заголовок календаря доставки".into());
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
    Ok(slots)
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
