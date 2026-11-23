use sim_radio::dielectric::Medium;
use sim_radio::timed::{AttemptLink, Outcome, Profile, Protocol, TimedChannel};
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
