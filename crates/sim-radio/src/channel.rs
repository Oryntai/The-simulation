//! Пакетный канал «устройство ↔ телефон» для совместной симуляции с прошивкой.
//!
//! * Bluetooth LE: обмен только в моменты «событий соединения» с периодом T (7.5 мс…4 с,
//!   Core Spec 5.x, Vol 6 Part B §4.5). Неподтверждённый пакет повторяется на следующем
//!   событии. Если подряд нет ни одного успешного обмена дольше «таймаута надзора» —
//!   соединение рвётся, дальше переподключение через рекламу (интервал задаём).
//! * Wi-Fi: доступ к эфиру с ожиданием (оценка 0.1–1.5 мс), до 7 повторов (dot11 RetryLimit),
//!   потом пакет теряется.
//!
//! Случайность — свой детерминированный генератор (одинаковый результат при одном seed).

use std::collections::VecDeque;

/// Протокол канала.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Protocol {
    Ble {
        interval_s: f64,
        supervision_timeout_s: f64,
        /// Период рекламы при переподключении.
        adv_interval_s: f64,
    },
    Wifi,
}

impl Protocol {
    /// Типовое соединение BLE для управления: 15 мс, таймаут 0.5 с, реклама 100 мс.
    pub fn ble_default() -> Self {
        Protocol::Ble {
            interval_s: 0.015,
            supervision_timeout_s: 0.5,
            adv_interval_s: 0.1,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Dir {
    DeviceToPhone,
    PhoneToDevice,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Delivery {
    pub dir: Dir,
    pub bytes: Vec<u8>,
    pub rssi_dbm: f64,
    pub latency_s: f64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Stats {
    pub sent: u64,
    pub delivered: u64,
    pub dropped: u64,
    pub disconnects: u64,
    pub latency_sum_s: f64,
    pub latency_max_s: f64,
}

impl Stats {
    pub fn mean_latency_s(&self) -> f64 {
        if self.delivered == 0 {
            0.0
        } else {
            self.latency_sum_s / self.delivered as f64
        }
    }

    pub fn loss_fraction(&self) -> f64 {
        if self.sent == 0 {
            0.0
        } else {
            self.dropped as f64 / self.sent as f64
        }
    }
}

/// Детерминированный генератор xorshift64*.
#[derive(Clone, Debug)]
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Self {
        Self(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1)
    }

    /// Равномерно в [0, 1).
    pub fn uniform(&mut self) -> f64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        let x = self.0.wrapping_mul(0x2545_F491_4F6C_DD1D);
        (x >> 11) as f64 / (1u64 << 53) as f64
    }
}

#[derive(Clone, Debug)]
struct Pending {
    dir: Dir,
    bytes: Vec<u8>,
    sent_at: f64,
    tries: u32,
    due: f64,
}

/// Канал между устройством и телефоном.
#[derive(Clone, Debug)]
pub struct PacketChannel {
    pub protocol: Protocol,
    per: f64,
    rssi_dbm: f64,
    rng: Rng,
    queue: VecDeque<Pending>,
    next_event: f64,
    connected: bool,
    last_ok: f64,
    pub stats: Stats,
}

/// Сколько пакетов в каждую сторону успевает пройти за одно событие BLE (оценка).
const BLE_PACKETS_PER_EVENT: usize = 4;
/// dot11LongRetryLimit/ShortRetryLimit по умолчанию — 7 попыток.
const WIFI_RETRY_LIMIT: u32 = 7;

impl PacketChannel {
    pub fn new(protocol: Protocol, seed: u64) -> Self {
        Self {
            protocol,
            per: 0.0,
            rssi_dbm: -60.0,
            rng: Rng::new(seed),
            queue: VecDeque::new(),
            next_event: 0.0,
            connected: true,
            last_ok: 0.0,
            stats: Stats::default(),
        }
    }

    /// Текущее качество линии: доля потерь одной попытки и уровень на приёмнике.
    pub fn set_link(&mut self, per: f64, rssi_dbm: f64) {
        self.per = per.clamp(0.0, 1.0);
        self.rssi_dbm = rssi_dbm;
    }

    pub fn connected(&self) -> bool {
        self.connected
    }

    pub fn queued(&self) -> usize {
        self.queue.len()
    }

    /// Поставить пакет в очередь. В разорванном BLE-соединении пакет сразу теряется
    /// (стек телефона/устройства не держит очередь без соединения).
    pub fn send(&mut self, now: f64, dir: Dir, bytes: &[u8]) {
        self.stats.sent += 1;
        if !self.connected {
            self.stats.dropped += 1;
            return;
        }
        let due = match self.protocol {
            Protocol::Ble { .. } => now,
            Protocol::Wifi => now + self.wifi_access_delay(),
        };
        self.queue.push_back(Pending {
            dir,
            bytes: bytes.to_vec(),
            sent_at: now,
            tries: 0,
            due,
        });
    }

    fn wifi_access_delay(&mut self) -> f64 {
        1e-4 + 1.4e-3 * self.rng.uniform()
    }

    fn deliver(&mut self, p: Pending, at: f64, out: &mut Vec<Delivery>) {
        let latency = at - p.sent_at;
        self.stats.delivered += 1;
        self.stats.latency_sum_s += latency;
        self.stats.latency_max_s = self.stats.latency_max_s.max(latency);
        out.push(Delivery {
            dir: p.dir,
            bytes: p.bytes,
            rssi_dbm: self.rssi_dbm,
            latency_s: latency,
        });
    }

    /// Прокрутить канал до момента `now`, вернуть доставленные пакеты.
    pub fn advance(&mut self, now: f64) -> Vec<Delivery> {
        let mut out = Vec::new();
        match self.protocol {
            Protocol::Ble {
                interval_s,
                supervision_timeout_s,
                adv_interval_s,
            } => {
                while self.next_event <= now {
                    let t = self.next_event;
                    if self.connected {
                        // Пустой обмен (keep-alive) тоже подтверждает связь.
                        let mut any_ok = self.rng.uniform() >= self.per;
                        for dir in [Dir::DeviceToPhone, Dir::PhoneToDevice] {
                            let mut sent = 0;
                            let mut i = 0;
                            while i < self.queue.len() && sent < BLE_PACKETS_PER_EVENT {
                                if self.queue[i].dir != dir || self.queue[i].sent_at > t {
                                    i += 1;
                                    continue;
                                }
                                sent += 1;
                                self.queue[i].tries += 1;
                                if self.rng.uniform() >= self.per {
                                    any_ok = true;
                                    let p = self.queue.remove(i).expect("индекс в очереди");
                                    self.deliver(p, t, &mut out);
                                } else {
                                    // Протокол не даёт обгонять: следующий пакет того же
                                    // направления ждёт, пока этот не пройдёт.
                                    break;
                                }
                            }
                        }
                        if any_ok {
                            self.last_ok = t;
                        } else if t - self.last_ok > supervision_timeout_s {
                            self.connected = false;
                            self.stats.disconnects += 1;
                            self.stats.dropped += self.queue.len() as u64;
                            self.queue.clear();
                        }
                        self.next_event = t + interval_s;
                    } else {
                        // Реклама: телефон должен услышать пакет и ответить запросом.
                        if self.rng.uniform() >= self.per && self.rng.uniform() >= self.per {
                            self.connected = true;
                            self.last_ok = t;
                            // Первое событие соединения — через 1.25 мс + окно (оценка: 1 интервал).
                            self.next_event = t + interval_s;
                        } else {
                            self.next_event = t + adv_interval_s;
                        }
                    }
                }
            }
            Protocol::Wifi => {
                let mut i = 0;
                while i < self.queue.len() {
                    if self.queue[i].due > now {
                        i += 1;
                        continue;
                    }
                    let t = self.queue[i].due;
                    self.queue[i].tries += 1;
                    if self.rng.uniform() >= self.per {
                        let p = self.queue.remove(i).expect("индекс в очереди");
                        self.deliver(p, t, &mut out);
                    } else if self.queue[i].tries >= WIFI_RETRY_LIMIT {
                        self.queue.remove(i);
                        self.stats.dropped += 1;
                    } else {
                        let d = self.wifi_access_delay();
                        self.queue[i].due = t + d;
                    }
                }
            }
        }
        out
    }
}

/// Средняя задержка одиночного пакета в BLE при равномерном моменте отправки:
/// T/2 (ждём событие) + T·p/(1 − p) (повторы с вероятностью неудачи p). Аналитика для сверки.
pub fn ble_mean_latency_analytic(interval_s: f64, per: f64) -> f64 {
    interval_s / 2.0 + interval_s * per / (1.0 - per)
}
