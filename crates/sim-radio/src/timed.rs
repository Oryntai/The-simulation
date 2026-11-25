//! Пакетная модель с целочисленным временем. Параметры радиолинии и пределы
//! очереди задаёт вызывающий код по измерениям, паспорту или оценке.

use std::collections::{BTreeMap, VecDeque};
use std::fmt;

use sim_signal::{Rng, Time};

use crate::dielectric::{Medium, Permittivity, interface_loss_db};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Protocol {
    Ble { interval_ns: u64 },
    Wifi { attempt_ns: u64 },
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Profile {
    pub protocol: Protocol,
    pub loss: f64,
    pub rssi_dbm: i8,
    pub attempts: u32,
    pub capacity: usize,
    pub max_bytes: usize,
}

/// Условия одной попытки. Потеря 1 означает отсутствие связи в этом слоте.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AttemptLink {
    pub loss: f64,
    pub rssi_dbm: i8,
}

impl AttemptLink {
    /// Бюджет трассы по нормали к поверхности. Ниже чувствительности
    /// вероятность потери равна 1, выше сохраняются базовые потери канала.
    pub fn through_layer(
        self,
        medium: Medium,
        depth_m: f64,
        frequency_hz: f64,
        sensitivity_dbm: f64,
    ) -> Result<Self, String> {
        if ![depth_m, frequency_hz, sensitivity_dbm]
            .into_iter()
            .all(f64::is_finite)
            || frequency_hz <= 0.0
            || depth_m < 0.0
            || ((medium == Medium::Air) != (depth_m == 0.0))
            || !(0.0..=1.0).contains(&self.loss)
            || matches!(medium, Medium::FreshWater { t_c } | Medium::SeaWater { t_c }
                if !t_c.is_finite())
        {
            return Err("недопустимые параметры среды попытки".into());
        }
        let attenuation = if medium == Medium::Air {
            0.0
        } else {
            let material = medium.permittivity(frequency_hz);
            material.attenuation_db_per_m(frequency_hz) * depth_m
                + interface_loss_db(Permittivity::VACUUM, material)
        };
        let rx = f64::from(self.rssi_dbm) - attenuation;
        if !rx.is_finite() || attenuation < 0.0 {
            return Err("некорректные потери слоя радиолинии".into());
        }
        Ok(Self {
            loss: if rx < sensitivity_dbm { 1.0 } else { self.loss },
            rssi_dbm: rx.round().clamp(i8::MIN.into(), i8::MAX.into()) as i8,
        })
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Outcome {
    Delivered { rssi_dbm: i8 },
    Queued,
    Retry,
    Lost,
    Disconnected,
    Overflow,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PacketEvent {
    pub id: u64,
    pub sent_at: Time,
    pub at: Time,
    pub attempt: u32,
    pub bytes: Vec<u8>,
    pub outcome: Outcome,
}

#[derive(Clone, Debug)]
pub struct TimedChannel {
    profile: Profile,
    rng: Rng,
    pending: BTreeMap<(Time, u64), PacketEvent>,
    now: Time,
    next_id: u64,
    connected: bool,
}

impl TimedChannel {
    pub fn new(profile: Profile, rng: Rng) -> Result<Self, String> {
        let period = match profile.protocol {
            Protocol::Ble { interval_ns } => interval_ns,
            Protocol::Wifi { attempt_ns } => attempt_ns,
        };
        if period == 0
            || !(0.0..=1.0).contains(&profile.loss)
            || profile.capacity == 0
            || profile.max_bytes == 0
            || profile.attempts == 0
        {
            return Err("некорректный профиль пакетного канала".into());
        }
        Ok(Self {
            profile,
            rng,
            pending: BTreeMap::new(),
            now: Time::ZERO,
            next_id: 0,
            connected: true,
        })
    }

    pub fn next_event(&self) -> Option<Time> {
        self.pending.first_key_value().map(|((at, _), _)| *at)
    }

    pub fn send(&mut self, at: Time, bytes: Vec<u8>) -> Result<PacketEvent, String> {
        self.send_after(at, bytes, 0)
    }

    pub fn send_after(
        &mut self,
        at: Time,
        bytes: Vec<u8>,
        hold_ns: u64,
    ) -> Result<PacketEvent, String> {
        if bytes.is_empty() || bytes.len() > self.profile.max_bytes {
            return Err("размер пакета выше предела линии".into());
        }
        self.check_time(at)?;
        if self.next_event().is_some_and(|due| due < at) {
            return Err("перед отправкой нужно обработать очередь канала".into());
        }
        let id = self.next_id;
        let mut event = PacketEvent {
            id,
            sent_at: at,
            at,
            attempt: 0,
            bytes,
            outcome: Outcome::Disconnected,
        };
        if self.connected {
            event.outcome = if self.pending.len() == self.profile.capacity {
                Outcome::Overflow
            } else {
                let ready =
                    at.0.checked_add(hold_ns)
                        .map(Time)
                        .ok_or("ожидание пакета вне шкалы времени")?;
                event.at = self.first_slot(ready)?;
                Outcome::Queued
            };
        }
        self.next_id = id.checked_add(1).ok_or("исчерпаны номера пакетов")?;
        self.now = at;
        if event.outcome == Outcome::Queued {
            self.pending.insert((event.at, id), event.clone());
        }
        Ok(event)
    }

    /// Разрыв сбрасывает пакеты в полёте. Восстановление начинает с пустой очереди.
    pub fn set_connected(&mut self, at: Time, connected: bool) -> Result<Vec<PacketEvent>, String> {
        self.check_time(at)?;
        if self.next_event().is_some_and(|due| due < at) {
            return Err("перед изменением связи нужно обработать очередь канала".into());
        }
        self.connected = connected;
        self.now = at;
        if connected {
            return Ok(Vec::new());
        }
        Ok(std::mem::take(&mut self.pending)
            .into_values()
            .map(|mut event| {
                event.at = at;
                event.outcome = Outcome::Disconnected;
                event
            })
            .collect())
    }

    pub fn advance(&mut self, until: Time) -> Result<Vec<PacketEvent>, String> {
        self.advance_with_link(until, |_, _, link| Ok(link))
    }

    /// Переоценивает линию перед каждой попыткой по её времени и длине пакета.
    /// При ошибке оценки очередь и зерно остаются в состоянии до вызова.
    pub fn advance_with_link(
        &mut self,
        until: Time,
        link_at: impl FnMut(Time, usize, AttemptLink) -> Result<AttemptLink, String>,
    ) -> Result<Vec<PacketEvent>, String> {
        let mut staged = self.clone();
        let events = staged.advance_inner(until, link_at)?;
        *self = staged;
        Ok(events)
    }

    fn advance_inner(
        &mut self,
        until: Time,
        mut link_at: impl FnMut(Time, usize, AttemptLink) -> Result<AttemptLink, String>,
    ) -> Result<Vec<PacketEvent>, String> {
        self.check_time(until)?;
        let mut events = Vec::new();
        while let Some((_, packet)) = self
            .pending
            .first_key_value()
            .filter(|(_, p)| p.at <= until)
        {
            let link = link_at(
                packet.at,
                packet.bytes.len(),
                AttemptLink {
                    loss: self.profile.loss,
                    rssi_dbm: self.profile.rssi_dbm,
                },
            )?;
            if !(0.0..=1.0).contains(&link.loss) {
                return Err("вероятность потери попытки вне диапазона [0, 1]".into());
            }
            let delivered = self.rng.uniform() >= link.loss;
            let retry = if !delivered && packet.attempt + 1 < self.profile.attempts {
                Some(self.first_slot(packet.at)?)
            } else {
                None
            };
            let (_, mut event) = self.pending.pop_first().expect("есть событие");
            self.now = event.at;
            event.attempt += 1;
            event.outcome = if delivered {
                Outcome::Delivered {
                    rssi_dbm: link.rssi_dbm,
                }
            } else if let Some(next) = retry {
                let mut queued = event.clone();
                queued.at = next;
                self.pending.insert((next, queued.id), queued);
                Outcome::Retry
            } else {
                Outcome::Lost
            };
            events.push(event);
        }
        self.now = until;
        Ok(events)
    }

    fn check_time(&self, at: Time) -> Result<(), String> {
        if at < self.now {
            Err("время канала пошло назад".into())
        } else {
            Ok(())
        }
    }

    fn first_slot(&self, at: Time) -> Result<Time, String> {
        // BLE ждёт строго следующего события соединения. Wi-Fi ждёт
        // полный заданный интервал доступа и передачи.
        let delay = match self.profile.protocol {
            Protocol::Ble { interval_ns } => interval_ns - at.0 % interval_ns,
            Protocol::Wifi { attempt_ns } => attempt_ns,
        };
        at.0.checked_add(delay)
            .map(Time)
            .ok_or_else(|| "доставка пакета вне шкалы времени".into())
    }
}

// ESP-IDF v6.1.1, components/esp_wifi/include/esp_now.h: ESP_NOW_ETH_ALEN,
// ESP_NOW_MAX_TOTAL_PEER_NUM, ESP_NOW_MAX_DATA_LEN и ESP_NOW_MAX_DATA_LEN_V2.
pub type EspNowAddress = [u8; 6];
const ESP_NOW_MAX_PEERS: usize = 20;
const ESP_NOW_V1_LIMIT: usize = 250;
const ESP_NOW_V2_LIMIT: usize = 1470;

// ESP-IDF v6.1.1, docs/en/api-reference/network/esp_now.rst, Frame Format и Add Paired Device.
pub const ESP_NOW_BROADCAST: EspNowAddress = [0xff; 6];
const ESP_NOW_MAX_CHANNEL: u8 = 14;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EspNowVersion {
    V1,
    V2,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
/// Интерфейсы ESP-NOW из ESP-IDF v6.1.1, Add Paired Device.
pub enum EspNowInterface {
    Station,
    SoftAp,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EspNowPeer {
    pub address: EspNowAddress,
    pub interface: EspNowInterface,
    /// `esp_now_peer_info_t.channel`: 0 означает текущий канал Wi-Fi.
    pub channel: u8,
    pub encrypted: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EspNowProfile {
    pub version: EspNowVersion,
    pub wifi_channel: u8,
    pub station_enabled: bool,
    pub soft_ap_enabled: bool,
    /// Модельная ёмкость FIFO, включая активную передачу и ожидание ответа.
    /// Руководство ESP-IDF, Send ESP-NOW Data, рекомендует ждать предыдущий
    /// callback. Ёмкость очереди и длительность callback задаются постановкой.
    pub queue_capacity: Option<usize>,
    /// Время одной попытки вместе с обработкой MAC в постановке. ESP-IDF
    /// v6.1.1 не задаёт для этой модели ни длительность, ни лимит попыток.
    pub attempt_ns: Option<u64>,
    /// Лимит unicast, включая первую попытку.
    pub mac_attempts: Option<u32>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EspNowConfirmation {
    Driver,
    /// Политика приложения из ESP-IDF, Send ESP-NOW Data. Таймер начинается
    /// после итогового MAC-события. Число передач включает первую.
    Application {
        timeout_ns: Option<u64>,
        transmissions: Option<u32>,
    },
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EspNowLink {
    pub frame: AttemptLink,
    /// Канал приёмника на моделируемом маршруте. `None` означает его отсутствие.
    pub receiver_channel: Option<u8>,
    /// Нужна при приёме кадра длиннее лимита v1.
    pub receiver_version: Option<EspNowVersion>,
    /// Условная вероятность потери MAC ACK после приёма unicast-кадра.
    pub mac_ack_loss: Option<f64>,
}

#[derive(Clone, Copy, Debug)]
pub struct EspNowAttempt<'a> {
    pub id: u64,
    pub destination: EspNowAddress,
    pub at: Time,
    pub transmission: u32,
    pub mac_attempt: u32,
    pub bytes: &'a [u8],
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum EspNowEventKind {
    /// Приём на стороне драйвера. Повторы одной передачи фильтруются по
    /// IEEE 802.11-2020 §10.3.2.14. Повтор приложения сохраняет `id`, а его
    /// обработка задаётся вызывающим кодом (ESP-IDF, Send ESP-NOW Data).
    Received {
        bytes: Vec<u8>,
        rssi_dbm: i8,
    },
    MacRetry,
    /// Отправитель получил MAC ACK. Обработка пакета приложением неизвестна.
    MacConfirmed,
    MacFailed,
    /// Окончание отправки в режиме без MAC ACK, RFC 9119 §3.1.1.
    BroadcastSent,
    ApplicationRetry,
    ApplicationConfirmed,
    ApplicationTimedOut,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EspNowEvent {
    pub id: u64,
    pub destination: EspNowAddress,
    pub at: Time,
    /// Текущая передача приложения, начиная с 1.
    pub transmission: u32,
    /// Сколько MAC-попыток исполнено в текущей передаче.
    pub mac_attempt: u32,
    pub kind: EspNowEventKind,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum EspNowError {
    NotAnalyzed(&'static str),
    Invalid(&'static str),
    // Смысл ошибок ESP_ERR_ESPNOW_* из esp_now.h. NoMemory соответствует
    // заполнению заданной очереди модели, а Full — таблицы адресатов.
    NoMemory,
    Full,
    Exists,
    NotFound,
    Interface,
    Channel,
    UnexpectedAck,
    Timing(String),
}

impl fmt::Display for EspNowError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotAnalyzed(reason) => write!(f, "ESP-NOW не проанализирован: {reason}"),
            Self::Invalid(reason) => write!(f, "неверный вход ESP-NOW: {reason}"),
            Self::NoMemory => f.write_str("очередь ESP-NOW заполнена"),
            Self::Full => f.write_str("таблица адресатов ESP-NOW заполнена"),
            Self::Exists => f.write_str("адресат ESP-NOW уже зарегистрирован"),
            Self::NotFound => f.write_str("адресат ESP-NOW не зарегистрирован"),
            Self::Interface => f.write_str("интерфейс адресата ESP-NOW выключен"),
            Self::Channel => f.write_str("канал адресата ESP-NOW отличается от текущего"),
            Self::UnexpectedAck => f.write_str("нет передачи ESP-NOW, ожидающей этот ответ"),
            Self::Timing(reason) => f.write_str(reason),
        }
    }
}

impl std::error::Error for EspNowError {}

#[derive(Clone, Debug)]
struct EspNowApplication {
    timeout_ns: u64,
    transmissions: u32,
    state: EspNowAckState,
}

#[derive(Clone, Copy, Debug)]
enum EspNowAckState {
    AwaitMac,
    Waiting(Time),
    Confirmed,
}

#[derive(Clone, Debug)]
struct EspNowTx {
    id: u64,
    destination: EspNowAddress,
    bytes: Vec<u8>,
    mac_limit: u32,
    application: Option<EspNowApplication>,
    transmission: u32,
    mac_attempt: u32,
    received_in: Option<u32>,
}

impl EspNowTx {
    fn event(&self, at: Time, kind: EspNowEventKind) -> EspNowEvent {
        EspNowEvent {
            id: self.id,
            destination: self.destination,
            at,
            transmission: self.transmission,
            mac_attempt: self.mac_attempt,
            kind,
        }
    }
}

/// Очередь одного отправителя с одним активным пакетом. После завершения
/// MAC callback следующий пакет может занять канал в тот же момент модели.
/// При ожидании ответа приложения FIFO удерживает пакет до ответа или тайм-аута.
/// Одна оценка линии описывает одного приёмника, в том числе для broadcast.
/// Результаты относятся к пакетной модели. Эфирные коллизии и задержки SDK
/// входят в параметры постановки и требуют независимого замера.
#[derive(Clone, Debug)]
pub struct EspNowChannel {
    profile: EspNowProfile,
    capacity: usize,
    radio: TimedChannel,
    peers: BTreeMap<EspNowAddress, EspNowPeer>,
    queue: VecDeque<EspNowTx>,
    now: Time,
    next_id: u64,
}

impl EspNowChannel {
    pub fn new(profile: EspNowProfile, rng: Rng) -> Result<Self, EspNowError> {
        if !(1..=ESP_NOW_MAX_CHANNEL).contains(&profile.wifi_channel)
            || profile.queue_capacity == Some(0)
            || profile.attempt_ns == Some(0)
            || profile.mac_attempts == Some(0)
        {
            return Err(EspNowError::Invalid(
                "канал Wi-Fi, ёмкость, время или лимит попыток",
            ));
        }
        let capacity = profile
            .queue_capacity
            .ok_or(EspNowError::NotAnalyzed("нет ёмкости очереди из данных"))?;
        let attempt_ns = profile.attempt_ns.ok_or(EspNowError::NotAnalyzed(
            "нет времени MAC-попытки из данных",
        ))?;
        let max_bytes = match profile.version {
            EspNowVersion::V1 => ESP_NOW_V1_LIMIT,
            EspNowVersion::V2 => ESP_NOW_V2_LIMIT,
        };
        let radio = TimedChannel::new(
            Profile {
                protocol: Protocol::Wifi { attempt_ns },
                // Одна попытка в транспорте, её параметры всегда подаёт оценка линии.
                loss: 1.0,
                rssi_dbm: i8::MIN,
                attempts: 1,
                capacity: 1,
                max_bytes,
            },
            rng,
        )
        .map_err(EspNowError::Timing)?;
        Ok(Self {
            profile,
            capacity,
            radio,
            peers: BTreeMap::new(),
            queue: VecDeque::new(),
            now: Time::ZERO,
            next_id: 0,
        })
    }

    pub fn add_peer(&mut self, peer: EspNowPeer) -> Result<(), EspNowError> {
        if peer.channel > ESP_NOW_MAX_CHANNEL {
            return Err(EspNowError::Invalid("канал адресата вне диапазона 0–14"));
        }
        if peer.address == ESP_NOW_BROADCAST {
            if peer.encrypted {
                // ESP-IDF, Security: групповые action-кадры не шифруются.
                return Err(EspNowError::Invalid("broadcast с шифрованием"));
            }
        } else if peer.address[0] & 1 != 0 {
            // RFC 9542 §2.1.1: младший бит первого октета — признак группы.
            return Err(EspNowError::NotAnalyzed("групповой адрес кроме broadcast"));
        } else if peer.encrypted {
            return Err(EspNowError::NotAnalyzed("состояние CCMP и ключей адресата"));
        }
        if self.peers.contains_key(&peer.address) {
            return Err(EspNowError::Exists);
        }
        if self.peers.len() == ESP_NOW_MAX_PEERS {
            return Err(EspNowError::Full);
        }
        self.peers.insert(peer.address, peer);
        Ok(())
    }

    pub fn next_event(&self) -> Option<Time> {
        self.radio.next_event().or_else(|| {
            let EspNowAckState::Waiting(at) = self.queue.front()?.application.as_ref()?.state
            else {
                return None;
            };
            Some(at)
        })
    }

    /// Ставит пакет в FIFO и возвращает его номер. Это результат постановки
    /// в очередь, а итог отправки поступает через `advance_with_link`.
    pub fn send(
        &mut self,
        at: Time,
        destination: EspNowAddress,
        bytes: Vec<u8>,
        confirmation: EspNowConfirmation,
    ) -> Result<u64, EspNowError> {
        self.check_time(at)?;
        if bytes.is_empty() || bytes.len() > self.radio.profile.max_bytes {
            return Err(EspNowError::Invalid("длина пакета для версии отправителя"));
        }
        let peer = self.peers.get(&destination).ok_or(EspNowError::NotFound)?;
        let enabled = match peer.interface {
            EspNowInterface::Station => self.profile.station_enabled,
            EspNowInterface::SoftAp => self.profile.soft_ap_enabled,
        };
        if !enabled {
            return Err(EspNowError::Interface);
        }
        if peer.channel != 0 && peer.channel != self.profile.wifi_channel {
            return Err(EspNowError::Channel);
        }
        let broadcast = destination == ESP_NOW_BROADCAST;
        let application = match confirmation {
            EspNowConfirmation::Driver => None,
            EspNowConfirmation::Application { .. } if broadcast => {
                return Err(EspNowError::Invalid(
                    "ожидание ответа требует unicast-адресата",
                ));
            }
            EspNowConfirmation::Application {
                timeout_ns,
                transmissions,
            } => {
                if timeout_ns == Some(0) || transmissions == Some(0) {
                    return Err(EspNowError::Invalid("нулевой тайм-аут или лимит передач"));
                }
                Some(EspNowApplication {
                    timeout_ns: timeout_ns.ok_or(EspNowError::NotAnalyzed(
                        "нет тайм-аута ответа приложения из данных",
                    ))?,
                    transmissions: transmissions.ok_or(EspNowError::NotAnalyzed(
                        "нет лимита передач приложения из данных",
                    ))?,
                    state: EspNowAckState::AwaitMac,
                })
            }
        };
        let mac_limit = if broadcast {
            // Одна отправка в модели без повтора по MAC ACK (RFC 9119 §3.1.1).
            1
        } else {
            self.profile
                .mac_attempts
                .ok_or(EspNowError::NotAnalyzed("нет лимита MAC-попыток из данных"))?
        };
        if self.queue.len() == self.capacity {
            return Err(EspNowError::NoMemory);
        }
        let id = self.next_id;
        let next_id = id
            .checked_add(1)
            .ok_or(EspNowError::Invalid("исчерпаны номера пакетов"))?;
        if self.queue.is_empty() {
            self.radio
                .send(at, bytes.clone())
                .map_err(EspNowError::Timing)?;
        }
        self.queue.push_back(EspNowTx {
            id,
            destination,
            bytes,
            mac_limit,
            application,
            transmission: 1,
            mac_attempt: 0,
            received_in: None,
        });
        self.next_id = next_id;
        self.now = at;
        Ok(id)
    }

    /// Принимает отдельный ответ приложения, уже доставленный обратным путём.
    /// Ответ сверяется по адресу и номеру. На отметке тайм-аута он уже просрочен.
    pub fn acknowledge(
        &mut self,
        at: Time,
        from: EspNowAddress,
        id: u64,
    ) -> Result<EspNowEvent, EspNowError> {
        self.check_time(at)?;
        let tx = self.queue.front_mut().ok_or(EspNowError::UnexpectedAck)?;
        if tx.id != id || tx.destination != from || tx.received_in.is_none() {
            return Err(EspNowError::UnexpectedAck);
        }
        let event = tx.event(at, EspNowEventKind::ApplicationConfirmed);
        let app = tx.application.as_mut().ok_or(EspNowError::UnexpectedAck)?;
        match app.state {
            // Ответ может прийти после приёма данных, пока продолжаются MAC-повторы.
            EspNowAckState::AwaitMac => app.state = EspNowAckState::Confirmed,
            EspNowAckState::Waiting(deadline) if at < deadline => self.finish(at)?,
            _ => return Err(EspNowError::UnexpectedAck),
        }
        self.now = at;
        Ok(event)
    }

    /// Продвигает модель включительно. Для подачи ответа приложения вызывающий
    /// код продвигается по `next_event`. При ошибке состояние и зерно сохраняются.
    pub fn advance_with_link(
        &mut self,
        until: Time,
        mut link_at: impl FnMut(EspNowAttempt<'_>) -> Result<EspNowLink, EspNowError>,
    ) -> Result<Vec<EspNowEvent>, EspNowError> {
        if until < self.now {
            return Err(EspNowError::Invalid("время ESP-NOW пошло назад"));
        }
        let mut staged = self.clone();
        let mut events = Vec::new();
        while let Some(at) = staged.next_event().filter(|at| *at <= until) {
            if staged.radio.next_event().is_some() {
                staged.mac_step(at, &mut link_at, &mut events)?;
            } else {
                staged.timeout(at, &mut events)?;
            }
        }
        staged.now = until;
        *self = staged;
        Ok(events)
    }

    fn mac_step(
        &mut self,
        at: Time,
        link_at: &mut impl FnMut(EspNowAttempt<'_>) -> Result<EspNowLink, EspNowError>,
        events: &mut Vec<EspNowEvent>,
    ) -> Result<(), EspNowError> {
        let tx = self.queue.front_mut().expect("у MAC-попытки есть пакет");
        tx.mac_attempt += 1;
        let link = link_at(EspNowAttempt {
            id: tx.id,
            destination: tx.destination,
            at,
            transmission: tx.transmission,
            mac_attempt: tx.mac_attempt,
            bytes: &tx.bytes,
        })?;
        if !(0.0..=1.0).contains(&link.frame.loss)
            || link
                .mac_ack_loss
                .is_some_and(|loss| !(0.0..=1.0).contains(&loss))
            || link
                .receiver_channel
                .is_some_and(|channel| !(1..=ESP_NOW_MAX_CHANNEL).contains(&channel))
        {
            return Err(EspNowError::Invalid(
                "вероятность потери или канал приёмника",
            ));
        }
        let mut frame = link.frame;
        // ESP-IDF, Send ESP-NOW Data: отсутствие получателя и разные каналы
        // приводят к неудаче unicast, даже если остальные потери равны нулю.
        if link.receiver_channel != Some(self.profile.wifi_channel) {
            frame.loss = 1.0;
        }
        let packet = self
            .radio
            .advance_inner(at, |_, _, _| Ok(frame))
            .map_err(EspNowError::Timing)?
            .pop()
            .expect("запланирована одна MAC-попытка");
        let received = if let Outcome::Delivered { rssi_dbm } = packet.outcome {
            // esp_now_get_version и Frame Format: для длинного v2 на v1
            // руководство допускает как отбрасывание, так и усечение.
            if tx.bytes.len() > ESP_NOW_V1_LIMIT && link.receiver_version != Some(EspNowVersion::V2)
            {
                return Err(EspNowError::NotAnalyzed(
                    "приём длинного v2-кадра на v1 или неизвестной версии",
                ));
            }
            if tx.received_in != Some(tx.transmission) {
                events.push(tx.event(
                    at,
                    EspNowEventKind::Received {
                        bytes: packet.bytes,
                        rssi_dbm,
                    },
                ));
                tx.received_in = Some(tx.transmission);
            }
            true
        } else {
            false
        };
        let outcome = if tx.destination == ESP_NOW_BROADCAST {
            EspNowEventKind::BroadcastSent
        } else {
            let confirmed = if received {
                let loss = link.mac_ack_loss.ok_or(EspNowError::NotAnalyzed(
                    "нет потерь обратного MAC ACK из данных",
                ))?;
                self.radio.rng.uniform() >= loss
            } else {
                false
            };
            if confirmed {
                EspNowEventKind::MacConfirmed
            } else if tx.mac_attempt < tx.mac_limit {
                self.radio
                    .send(at, tx.bytes.clone())
                    .map_err(EspNowError::Timing)?;
                events.push(tx.event(at, EspNowEventKind::MacRetry));
                return Ok(());
            } else {
                EspNowEventKind::MacFailed
            }
        };
        events.push(tx.event(at, outcome));
        if let Some(app) = tx
            .application
            .as_mut()
            .filter(|app| matches!(app.state, EspNowAckState::AwaitMac))
        {
            app.state =
                EspNowAckState::Waiting(Time(at.0.checked_add(app.timeout_ns).ok_or_else(
                    || EspNowError::Timing("тайм-аут ответа ESP-NOW вне шкалы времени".into()),
                )?));
            Ok(())
        } else {
            self.finish(at)
        }
    }

    fn timeout(&mut self, at: Time, events: &mut Vec<EspNowEvent>) -> Result<(), EspNowError> {
        let tx = self.queue.front_mut().expect("у тайм-аута есть пакет");
        let app = tx
            .application
            .as_mut()
            .expect("пакет ждёт ответа приложения");
        if tx.transmission == app.transmissions {
            events.push(tx.event(at, EspNowEventKind::ApplicationTimedOut));
            return self.finish(at);
        }
        app.state = EspNowAckState::AwaitMac;
        events.push(tx.event(at, EspNowEventKind::ApplicationRetry));
        self.radio
            .send(at, tx.bytes.clone())
            .map_err(EspNowError::Timing)?;
        tx.transmission += 1;
        tx.mac_attempt = 0;
        Ok(())
    }

    fn finish(&mut self, at: Time) -> Result<(), EspNowError> {
        if let Some(next) = self.queue.get(1) {
            self.radio
                .send(at, next.bytes.clone())
                .map_err(EspNowError::Timing)?;
        }
        self.queue.pop_front();
        Ok(())
    }

    fn check_time(&self, at: Time) -> Result<(), EspNowError> {
        if at < self.now {
            return Err(EspNowError::Invalid("время ESP-NOW пошло назад"));
        }
        if self.next_event().is_some_and(|due| due < at) {
            return Err(EspNowError::Invalid("сначала обработайте события ESP-NOW"));
        }
        Ok(())
    }
}
