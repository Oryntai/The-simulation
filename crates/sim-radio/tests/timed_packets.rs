use sim_radio::dielectric::Medium;
use sim_radio::timed::EspNowConfirmation::Driver;
use sim_radio::timed::EspNowEventKind::*;
use sim_radio::timed::{
    AttemptLink, ESP_NOW_BROADCAST, EspNowAddress, EspNowAttempt, EspNowChannel,
    EspNowConfirmation, EspNowError, EspNowEvent, EspNowEventKind, EspNowInterface, EspNowLink,
    EspNowPeer, EspNowProfile, EspNowVersion, Outcome, Profile, Protocol, TimedChannel,
};
use sim_signal::{Rng, Time};

fn profile(protocol: Protocol, loss: f64) -> Profile {
    // Синтетический стенд с двумя слотами и тремя попытками на пакет.
    Profile {
        protocol,
        loss,
        rssi_dbm: -64,
        capacity: 2,
        max_bytes: 3,
        attempts: 3,
    }
}

#[test]
fn слой_переоценивается_на_каждой_попытке() {
    // Синтетические слоты BLE, чувствительность и глубина для двух
    // противоположных исходов бюджета. 2,4 ГГц — диапазон BLE.
    let period = 7_500_000;
    let wet = Medium::FreshWater { t_c: 22.0 };
    let mut radio = TimedChannel::new(
        profile(
            Protocol::Ble {
                interval_ns: period,
            },
            0.0,
        ),
        Rng::stream(13, "стержень"),
    )
    .unwrap();
    let queued = radio.send(Time::ZERO, vec![1, 2]).unwrap();
    let mut fully_wet = radio.clone();
    let mut calls = Vec::new();
    let events = radio
        .advance_with_link(Time(3 * period), |at, len, link| {
            calls.push((at, len));
            let (medium, depth) = if at == Time(period) {
                (wet, 0.1)
            } else {
                (Medium::Air, 0.0)
            };
            link.through_layer(medium, depth, 2.4e9, -90.0)
        })
        .unwrap();
    assert_eq!(calls, [(Time(period), 2), (Time(2 * period), 2)]);
    assert_eq!(events[0].outcome, Outcome::Retry);
    assert_eq!(events[1].outcome, Outcome::Delivered { rssi_dbm: -64 });
    assert_eq!((events[1].id, events[1].attempt), (queued.id, 2));
    let events = fully_wet
        .advance_with_link(Time(3 * period), |_, _, link| {
            link.through_layer(wet, 0.1, 2.4e9, -90.0)
        })
        .unwrap();
    assert_eq!(
        events.iter().map(|e| &e.outcome).collect::<Vec<_>>(),
        [&Outcome::Retry, &Outcome::Retry, &Outcome::Lost]
    );
}

#[test]
fn неверная_среда_не_теряет_пакет_и_зерно() {
    let period = 1_000_000; // Синтетический Wi-Fi с равными интервалами.
    let mut radio = TimedChannel::new(
        profile(Protocol::Wifi { attempt_ns: period }, 0.5),
        Rng::stream(11, "каретка"),
    )
    .unwrap();
    radio.send(Time::ZERO, vec![1]).unwrap();
    radio.send(Time::ZERO, vec![2, 3]).unwrap();
    let mut intact = radio.clone();
    assert!(
        radio
            .advance_with_link(Time(period), |_, len, link| {
                Ok(AttemptLink {
                    loss: if len == 2 { f64::NAN } else { link.loss },
                    ..link
                })
            })
            .is_err()
    );
    assert_eq!(radio.next_event(), intact.next_event());
    assert_eq!(
        radio.advance(Time(3 * period)),
        intact.advance(Time(3 * period))
    );
}

#[test]
fn ble_exact_slots_and_capacity() {
    let mut radio = TimedChannel::new(
        profile(
            Protocol::Ble {
                interval_ns: 7_500_000,
            },
            0.0,
        ),
        Rng::stream(1, "линия"),
    )
    .unwrap();
    // Отправка 7 мс, слот 7.5 мс. Второй отправлен ровно на слоте и ждёт следующего.
    let first = radio.send(Time(7_000_000), vec![1, 2]).unwrap();
    let second = radio.send(Time(7_500_000), vec![3]).unwrap();
    assert_eq!((first.at, second.at), (Time(7_500_000), Time(15_000_000)));
    assert_eq!(
        radio.send(Time(7_500_000), vec![4]).unwrap().outcome,
        Outcome::Overflow
    );
    let events = radio.advance(Time(15_000_000)).unwrap();
    assert_eq!(
        events
            .iter()
            .map(|p| (
                p.id,
                p.sent_at,
                p.at,
                p.attempt,
                p.bytes.clone(),
                p.outcome.clone()
            ))
            .collect::<Vec<_>>(),
        [
            (
                0,
                Time(7_000_000),
                Time(7_500_000),
                1,
                vec![1, 2],
                Outcome::Delivered { rssi_dbm: -64 }
            ),
            (
                1,
                Time(7_500_000),
                Time(15_000_000),
                1,
                vec![3],
                Outcome::Delivered { rssi_dbm: -64 }
            ),
        ]
    );
}

#[test]
fn wifi_retries_disconnection_and_fork() {
    // 1 мс на попытку × 3 даёт исход на 3 мс после постановки в эфир.
    let p = profile(
        Protocol::Wifi {
            attempt_ns: 1_000_000,
        },
        1.0,
    );
    let mut radio = TimedChannel::new(p, Rng::stream(2, "линия")).unwrap();
    let queued = radio
        .send_after(Time(3_000_000), vec![0x42], 2_000_000)
        .unwrap();
    assert_eq!(queued.at, Time(6_000_000));
    let first = radio.advance(Time(6_000_000)).unwrap();
    assert_eq!(first[0].outcome, Outcome::Retry);
    let mut fork = radio.clone();
    let baseline = radio.advance(Time(8_000_000)).unwrap();
    assert_eq!(baseline, fork.clone().advance(Time(8_000_000)).unwrap());
    assert_eq!(
        baseline
            .iter()
            .map(|p| (p.at, p.attempt, p.outcome.clone()))
            .collect::<Vec<_>>(),
        [
            (Time(7_000_000), 2, Outcome::Retry),
            (Time(8_000_000), 3, Outcome::Lost)
        ]
    );
    let dropped = fork.set_connected(Time(6_500_000), false).unwrap();
    assert_eq!(
        (
            dropped[0].at,
            dropped[0].bytes.as_slice(),
            &dropped[0].outcome
        ),
        (Time(6_500_000), &[0x42][..], &Outcome::Disconnected)
    );
    assert!(fork.advance(Time(8_000_000)).unwrap().is_empty());
}

#[test]
fn invalid_profile_time_and_payload_are_errors() {
    let rng = Rng::stream(1, "линия");
    for loss in [f64::NAN, f64::INFINITY, -0.1, 1.1] {
        assert!(
            TimedChannel::new(profile(Protocol::Wifi { attempt_ns: 1 }, loss), rng.clone())
                .is_err()
        );
    }
    assert!(
        TimedChannel::new(profile(Protocol::Ble { interval_ns: 0 }, 0.0), rng.clone()).is_err()
    );
    let mut radio = TimedChannel::new(profile(Protocol::Wifi { attempt_ns: 1 }, 0.0), rng).unwrap();
    assert!(radio.send(Time::ZERO, vec![0; 4]).is_err());
    assert!(radio.send(Time(u64::MAX), vec![1]).is_err());
    let good = radio.send(Time(10), vec![2]).unwrap();
    assert_eq!(good.id, 0);
    assert!(radio.send(Time(9), vec![3]).is_err());
    assert!(radio.send(Time(13), vec![3]).is_err());
    assert_eq!(radio.advance(Time(13)).unwrap()[0].at, Time(11));
}

// Синтетический пакетный стенд. Адреса локально администрируемые (RFC 9542 §2.1.1).
const BLOCK: EspNowAddress = [2, 0, 0, 0, 0, 1];
const ROD: EspNowAddress = [2, 0, 0, 0, 0, 2];
const ATTEMPT_NS: u64 = 10;
const ACK_NS: u64 = 25;
const WIFI_CHANNEL: u8 = 6;
const APP_ACK: EspNowConfirmation = EspNowConfirmation::Application {
    timeout_ns: Some(ACK_NS),
    transmissions: Some(2),
};

fn esp_profile(version: EspNowVersion) -> EspNowProfile {
    EspNowProfile {
        version,
        wifi_channel: WIFI_CHANNEL,
        station_enabled: true,
        soft_ap_enabled: false,
        queue_capacity: Some(2),
        attempt_ns: Some(ATTEMPT_NS),
        mac_attempts: Some(2),
    }
}

fn peer(address: EspNowAddress) -> EspNowPeer {
    EspNowPeer {
        address,
        interface: EspNowInterface::Station,
        channel: 0,
        encrypted: false,
    }
}

fn esp_channel(version: EspNowVersion) -> EspNowChannel {
    let mut channel = EspNowChannel::new(esp_profile(version), Rng::stream(17, "блок")).unwrap();
    channel.add_peer(peer(BLOCK)).unwrap();
    channel
}

fn esp_link() -> EspNowLink {
    EspNowLink {
        frame: AttemptLink {
            loss: 0.0,
            rssi_dbm: -64,
        },
        receiver_channel: Some(WIFI_CHANNEL),
        receiver_version: Some(EspNowVersion::V2),
        mac_ack_loss: Some(0.0),
    }
}

fn received(bytes: Vec<u8>) -> EspNowEventKind {
    Received {
        bytes,
        rssi_dbm: esp_link().frame.rssi_dbm,
    }
}

fn trace(events: Vec<EspNowEvent>) -> Vec<(u64, Time, u32, u32, EspNowEventKind)> {
    events
        .into_iter()
        .map(|e| (e.id, e.at, e.transmission, e.mac_attempt, e.kind))
        .collect()
}

#[test]
fn esp_now_очередь_ждёт_mac_callback_и_проверяет_канал() {
    let mut channel = esp_channel(EspNowVersion::V1);
    channel
        .add_peer(EspNowPeer {
            channel: WIFI_CHANNEL,
            ..peer(ROD)
        })
        .unwrap();
    let first = channel.send(Time(100), BLOCK, vec![1], Driver).unwrap();
    let second = channel.send(Time(105), ROD, vec![2], Driver).unwrap();
    assert_eq!(
        channel.send(Time(105), ROD, vec![3], Driver),
        Err(EspNowError::NoMemory)
    );
    assert!(matches!(
        channel.send(Time(111), ROD, vec![3], Driver),
        Err(EspNowError::Invalid(_))
    ));
    assert_eq!(channel.next_event(), Some(Time(110)));

    let mut scheduled = Vec::new();
    let mut link = |attempt: EspNowAttempt<'_>| {
        scheduled.push((attempt.at.0, attempt.destination, attempt.bytes[0]));
        Ok(EspNowLink {
            receiver_channel: if attempt.at == Time(110) {
                Some(WIFI_CHANNEL + 1)
            } else {
                Some(WIFI_CHANNEL)
            },
            ..esp_link()
        })
    };
    let mut events = channel.advance_with_link(Time(119), &mut link).unwrap();
    assert_eq!(events.len(), 1);
    assert_eq!(
        channel.send(Time(119), ROD, vec![3], Driver),
        Err(EspNowError::NoMemory)
    );
    events.extend(channel.advance_with_link(Time(120), &mut link).unwrap());
    let third = channel.send(Time(120), BLOCK, vec![3], Driver).unwrap();
    events.extend(channel.advance_with_link(Time(140), link).unwrap());

    // Ручное расписание: 100 + 2×10 до callback, затем ещё по 10 на пакет.
    assert_eq!([first, second, third], [0, 1, 2]);
    assert_eq!(
        scheduled,
        [
            (110, BLOCK, 1),
            (120, BLOCK, 1),
            (130, ROD, 2),
            (140, BLOCK, 3)
        ]
    );
    assert_eq!(
        trace(events),
        [
            (first, Time(110), 1, 1, MacRetry),
            (first, Time(120), 1, 2, received(vec![1])),
            (first, Time(120), 1, 2, MacConfirmed),
            (second, Time(130), 1, 1, received(vec![2])),
            (second, Time(130), 1, 1, MacConfirmed),
            (third, Time(140), 1, 1, received(vec![3])),
            (third, Time(140), 1, 1, MacConfirmed),
        ]
    );
    assert_eq!(channel.next_event(), None);
}

#[test]
fn esp_now_ответ_приложения_возможен_при_потере_mac_ack() {
    let mut channel = esp_channel(EspNowVersion::V1);
    channel.add_peer(peer(ROD)).unwrap();
    let first = channel.send(Time::ZERO, BLOCK, vec![11], APP_ACK).unwrap();
    let second = channel.send(Time::ZERO, ROD, vec![22], Driver).unwrap();
    assert_eq!(
        channel.acknowledge(Time::ZERO, BLOCK, first),
        Err(EspNowError::UnexpectedAck)
    );
    let lost_ack = |_: EspNowAttempt<'_>| {
        Ok(EspNowLink {
            mac_ack_loss: Some(1.0),
            ..esp_link()
        })
    };
    let first_attempt = channel
        .advance_with_link(Time(ATTEMPT_NS), lost_ack)
        .unwrap();
    assert_eq!(
        trace(first_attempt),
        [
            (first, Time(10), 1, 1, received(vec![11])),
            (first, Time(10), 1, 1, MacRetry),
        ]
    );
    assert_eq!(
        channel.acknowledge(Time(15), ROD, first),
        Err(EspNowError::UnexpectedAck)
    );
    assert_eq!(
        channel.acknowledge(Time(15), BLOCK, second),
        Err(EspNowError::UnexpectedAck)
    );
    let mut waiting = channel.clone();
    let ack = channel.acknowledge(Time(15), BLOCK, first).unwrap();
    assert_eq!(
        trace(vec![ack]),
        [(first, Time(15), 1, 1, ApplicationConfirmed)]
    );
    assert_eq!(
        channel.acknowledge(Time(15), BLOCK, first),
        Err(EspNowError::UnexpectedAck)
    );
    assert_eq!(
        trace(channel.advance_with_link(Time(20), lost_ack).unwrap()),
        [(first, Time(20), 1, 2, MacFailed),]
    );
    assert_eq!(channel.next_event(), Some(Time(30)));
    assert_eq!(
        trace(
            channel
                .advance_with_link(Time(30), |_| Ok(esp_link()))
                .unwrap()
        ),
        [
            (second, Time(30), 1, 1, received(vec![22])),
            (second, Time(30), 1, 1, MacConfirmed),
        ]
    );

    assert_eq!(
        trace(waiting.advance_with_link(Time(20), lost_ack).unwrap()),
        [(first, Time(20), 1, 2, MacFailed),]
    );
    assert_eq!(waiting.next_event(), Some(Time(45)));
    assert_eq!(
        waiting.acknowledge(Time(45), BLOCK, first),
        Err(EspNowError::UnexpectedAck)
    );
    assert_eq!(
        waiting.acknowledge(Time(44), BLOCK, first).unwrap().kind,
        ApplicationConfirmed
    );
    assert_eq!(waiting.next_event(), Some(Time(54)));
}

#[test]
fn esp_now_mac_успех_не_закрывает_таймер_ответа_приложения() {
    let mut channel = esp_channel(EspNowVersion::V2);
    let id = channel.send(Time(8), BLOCK, vec![7], APP_ACK).unwrap();
    let mut fork = channel.clone();
    let mut slice = Vec::new();
    while let Some(at) = fork.next_event() {
        slice.extend(fork.advance_with_link(at, |_| Ok(esp_link())).unwrap());
    }
    let events = channel
        .advance_with_link(Time(78), |_| Ok(esp_link()))
        .unwrap();
    assert_eq!(events, slice);
    // При успехе с первой MAC-попытки цикл равен 10 + 25 нс. После двух
    // циклов дедлайн равен 8 + 2×35 = 78 нс. Номер и данные те же.
    assert_eq!(
        trace(events),
        [
            (id, Time(18), 1, 1, received(vec![7])),
            (id, Time(18), 1, 1, MacConfirmed),
            (id, Time(43), 1, 1, ApplicationRetry),
            (id, Time(53), 2, 1, received(vec![7])),
            (id, Time(53), 2, 1, MacConfirmed),
            (id, Time(78), 2, 1, ApplicationTimedOut),
        ]
    );
    assert_eq!(
        channel.acknowledge(Time(78), BLOCK, id),
        Err(EspNowError::UnexpectedAck)
    );
    assert_eq!(channel.next_event(), None);
}

#[test]
fn esp_now_broadcast_отправляется_без_подтверждения() {
    let mut profile = esp_profile(EspNowVersion::V2);
    profile.mac_attempts = None;
    let mut channel = EspNowChannel::new(profile, Rng::stream(17, "стержень")).unwrap();
    assert_eq!(
        channel.send(Time::ZERO, ESP_NOW_BROADCAST, vec![9], Driver),
        Err(EspNowError::NotFound)
    );
    channel.add_peer(peer(ESP_NOW_BROADCAST)).unwrap();
    assert!(matches!(
        channel.send(Time::ZERO, ESP_NOW_BROADCAST, vec![9], APP_ACK),
        Err(EspNowError::Invalid(_))
    ));
    let lost = channel
        .send(Time::ZERO, ESP_NOW_BROADCAST, vec![9], Driver)
        .unwrap();
    let heard = channel
        .send(Time::ZERO, ESP_NOW_BROADCAST, vec![10], Driver)
        .unwrap();
    let events = channel
        .advance_with_link(Time(20), |attempt| {
            Ok(EspNowLink {
                receiver_channel: if attempt.id == lost {
                    None
                } else {
                    Some(WIFI_CHANNEL)
                },
                mac_ack_loss: None,
                ..esp_link()
            })
        })
        .unwrap();
    assert!(
        events
            .iter()
            .all(|event| event.destination == ESP_NOW_BROADCAST)
    );
    assert_eq!(
        trace(events),
        [
            (lost, Time(10), 1, 1, BroadcastSent),
            (heard, Time(20), 1, 1, received(vec![10])),
            (heard, Time(20), 1, 1, BroadcastSent),
        ]
    );
    assert_eq!(channel.next_event(), None);
}

#[test]
fn esp_now_пределы_версий_и_неоднозначный_приём() {
    // Независимые граничные значения из esp_now.h и Frame Format ESP-IDF.
    for (version, limit) in [(EspNowVersion::V1, 250), (EspNowVersion::V2, 1470)] {
        let mut channel = esp_channel(version);
        for size in [0, limit + 1] {
            assert!(matches!(
                channel.send(Time::ZERO, BLOCK, vec![3; size], Driver),
                Err(EspNowError::Invalid(_))
            ));
        }
        let expected = vec![3; limit];
        let id = channel
            .send(Time::ZERO, BLOCK, expected.clone(), Driver)
            .unwrap();
        for receiver_version in [None, Some(EspNowVersion::V1)] {
            let mut fork = channel.clone();
            let result = fork.advance_with_link(Time(ATTEMPT_NS), |_| {
                Ok(EspNowLink {
                    receiver_version,
                    ..esp_link()
                })
            });
            if version == EspNowVersion::V1 {
                assert_eq!(
                    trace(result.unwrap()),
                    [
                        (id, Time(10), 1, 1, received(expected.clone())),
                        (id, Time(10), 1, 1, MacConfirmed),
                    ]
                );
            } else {
                assert!(matches!(result, Err(EspNowError::NotAnalyzed(_))));
                assert_eq!(fork.next_event(), Some(Time(10)));
            }
        }
        assert_eq!(
            trace(
                channel
                    .advance_with_link(Time(ATTEMPT_NS), |_| Ok(esp_link()))
                    .unwrap()
            ),
            [
                (id, Time(10), 1, 1, received(expected)),
                (id, Time(10), 1, 1, MacConfirmed),
            ]
        );
    }
    let mut v2 = esp_channel(EspNowVersion::V2);
    let id = v2.send(Time::ZERO, BLOCK, vec![3; 250], Driver).unwrap();
    assert_eq!(
        trace(
            v2.advance_with_link(Time(10), |_| Ok(EspNowLink {
                receiver_version: Some(EspNowVersion::V1),
                ..esp_link()
            }))
            .unwrap()
        ),
        [
            (id, Time(10), 1, 1, received(vec![3; 250])),
            (id, Time(10), 1, 1, MacConfirmed)
        ]
    );
}

#[test]
fn esp_now_таблица_и_параметры_передачи_проверяются() {
    let mut channel = esp_channel(EspNowVersion::V1);
    assert_eq!(channel.add_peer(peer(BLOCK)), Err(EspNowError::Exists));
    assert!(matches!(
        channel.add_peer(EspNowPeer {
            encrypted: true,
            ..peer(ROD)
        }),
        Err(EspNowError::NotAnalyzed(_))
    ));
    assert!(matches!(
        channel.add_peer(EspNowPeer {
            encrypted: true,
            ..peer(ESP_NOW_BROADCAST)
        }),
        Err(EspNowError::Invalid(_))
    ));
    channel
        .add_peer(EspNowPeer {
            channel: WIFI_CHANNEL + 1,
            ..peer(ROD)
        })
        .unwrap();
    assert_eq!(
        channel.send(Time::ZERO, ROD, vec![1], Driver),
        Err(EspNowError::Channel)
    );
    let other = [2, 0, 0, 0, 0, 3];
    channel
        .add_peer(EspNowPeer {
            interface: EspNowInterface::SoftAp,
            ..peer(other)
        })
        .unwrap();
    assert_eq!(
        channel.send(Time::ZERO, other, vec![1], Driver),
        Err(EspNowError::Interface)
    );
    assert!(matches!(
        channel.add_peer(EspNowPeer {
            channel: 15,
            ..peer(other)
        }),
        Err(EspNowError::Invalid(_))
    ));
    // ESP_NOW_MAX_TOTAL_PEER_NUM = 20 в esp_now.h. Первые три адресата уже есть.
    for n in 4..=20 {
        channel.add_peer(peer([2, 0, 0, 0, 0, n])).unwrap();
    }
    assert_eq!(
        channel.add_peer(peer(ESP_NOW_BROADCAST)),
        Err(EspNowError::Full)
    );
    assert_eq!(channel.next_event(), None);

    let profile = esp_profile(EspNowVersion::V1);
    let rng = Rng::stream(17, "блок");
    for invalid in [
        EspNowProfile {
            attempt_ns: Some(0),
            ..profile
        },
        EspNowProfile {
            wifi_channel: 0,
            ..profile
        },
    ] {
        assert!(matches!(
            EspNowChannel::new(invalid, rng.clone()),
            Err(EspNowError::Invalid(_))
        ));
    }
    for incomplete in [
        EspNowProfile {
            attempt_ns: None,
            ..profile
        },
        EspNowProfile {
            queue_capacity: None,
            ..profile
        },
    ] {
        assert!(matches!(
            EspNowChannel::new(incomplete, rng.clone()),
            Err(EspNowError::NotAnalyzed(_))
        ));
    }
    let mut unknown_retry = EspNowChannel::new(
        EspNowProfile {
            mac_attempts: None,
            ..profile
        },
        rng,
    )
    .unwrap();
    unknown_retry.add_peer(peer(BLOCK)).unwrap();
    assert!(matches!(
        unknown_retry.send(Time::ZERO, BLOCK, vec![1], Driver),
        Err(EspNowError::NotAnalyzed(_))
    ));
    for confirmation in [
        EspNowConfirmation::Application {
            timeout_ns: None,
            transmissions: Some(2),
        },
        EspNowConfirmation::Application {
            timeout_ns: Some(ACK_NS),
            transmissions: None,
        },
    ] {
        assert!(matches!(
            channel.send(Time::ZERO, BLOCK, vec![1], confirmation),
            Err(EspNowError::NotAnalyzed(_))
        ));
    }
}

#[test]
fn esp_now_ошибка_не_меняет_очередь_и_rng() {
    let mut channel = esp_channel(EspNowVersion::V1);
    channel.send(Time::ZERO, BLOCK, vec![1], Driver).unwrap();
    channel.send(Time::ZERO, BLOCK, vec![2], Driver).unwrap();
    let mut intact = channel.clone();
    let stochastic = |_: EspNowAttempt<'_>| {
        Ok(EspNowLink {
            frame: AttemptLink {
                loss: 0.25,
                ..esp_link().frame
            },
            mac_ack_loss: Some(0.2),
            ..esp_link()
        })
    };
    for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, 1.1] {
        assert!(matches!(
            channel.advance_with_link(Time(40), |attempt| {
                if attempt.at == Time(10) {
                    stochastic(attempt)
                } else {
                    Ok(EspNowLink {
                        mac_ack_loss: Some(invalid),
                        ..esp_link()
                    })
                }
            }),
            Err(EspNowError::Invalid(_))
        ));
        assert_eq!(channel.next_event(), Some(Time(10)));
    }
    assert!(matches!(
        channel.advance_with_link(Time(10), |_| Ok(EspNowLink {
            mac_ack_loss: None,
            ..esp_link()
        })),
        Err(EspNowError::NotAnalyzed(_))
    ));
    assert!(matches!(
        channel.advance_with_link(Time(10), |_| Ok(EspNowLink {
            frame: AttemptLink {
                loss: f64::NAN,
                ..esp_link().frame
            },
            ..esp_link()
        })),
        Err(EspNowError::Invalid(_))
    ));
    assert_eq!(
        channel.advance_with_link(Time(40), stochastic).unwrap(),
        intact.advance_with_link(Time(40), stochastic).unwrap()
    );
    assert!(channel.advance_with_link(Time(39), stochastic).is_err());

    let mut overflow = esp_channel(EspNowVersion::V1);
    let at = Time(u64::MAX - ATTEMPT_NS);
    assert!(matches!(
        overflow.send(Time(at.0 + 1), BLOCK, vec![1], Driver),
        Err(EspNowError::Timing(_))
    ));
    assert_eq!(overflow.send(at, BLOCK, vec![1], APP_ACK), Ok(0));
    assert!(matches!(
        overflow.advance_with_link(Time(u64::MAX), |_| Ok(esp_link())),
        Err(EspNowError::Timing(_))
    ));
    assert_eq!(overflow.next_event(), Some(Time(u64::MAX)));
}

#[test]
fn esp_now_потери_mac_сходятся_с_биномиальной_моделью() {
    // Синтетическая серия с двумя попытками и заданными потерями. Вероятность
    // обеих неудач вычисляется независимо как (1 − (1−p)(1−q))² = 0,16.
    let samples = 4096;
    let (p, q) = (0.25, 0.2);
    let failure = (1.0_f64 - (1.0 - p) * (1.0 - q)).powi(2);
    let mut channel = esp_channel(EspNowVersion::V1);
    let mut failures = 0;
    for i in 0..samples {
        let at = Time(i * 2 * ATTEMPT_NS);
        channel.send(at, BLOCK, vec![7], Driver).unwrap();
        let events = channel
            .advance_with_link(Time(at.0 + 2 * ATTEMPT_NS), |_| {
                Ok(EspNowLink {
                    frame: AttemptLink {
                        loss: p,
                        ..esp_link().frame
                    },
                    mac_ack_loss: Some(q),
                    ..esp_link()
                })
            })
            .unwrap();
        match events.last().unwrap().kind {
            MacFailed => failures += 1,
            MacConfirmed => {}
            ref kind => panic!("нет терминального исхода: {kind:?}"),
        }
    }
    let observed = f64::from(failures) / samples as f64;
    println!("Отказы MAC: {failures}/{samples} = {observed:.6}, расчёт {failure:.6}");
    // Пятикратная стандартная ошибка доли при биномиальном распределении.
    let tolerance = 5.0 * (failure * (1.0 - failure) / samples as f64).sqrt();
    assert!(
        (observed - failure).abs() < tolerance,
        "{failures}/{samples} = {observed}, эталон {failure}"
    );
}
