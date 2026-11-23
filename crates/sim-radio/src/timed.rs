//! Пакетная модель с целочисленным временем. Параметры радиолинии и пределы
//! очереди задаёт вызывающий код по измерениям, паспорту или оценке.

use std::collections::BTreeMap;

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
