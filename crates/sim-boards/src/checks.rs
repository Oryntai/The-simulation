//! Сквозной стенд с сохранёнными машинными образами. Эталонные формулы вычисляются
//! из параметров прошивок и даташитов, отдельно от исполнения и состояния регистров.

use crate::{BoardFault, BoardId, new_board};
use sim_verify::{Check, Tolerance};

struct Fixture {
    name: &'static str,
    ids: [&'static str; 5],
    board: BoardId,
    image: &'static [u8],
    led: u32,
    input: u32,
    pwm: u32,
    pwm_sample_hz: f64,
    watchdog_s: f64,
    channel_volts: f64,
    measurement_limit_s: f64,
    reset_check_s: f64,
    frequency: f64,
    blink_ticks: f64,
    pwm_high: f64,
    pwm_ticks: f64,
    adc_bins: f64,
    vref: f64,
    clock_reference: &'static str,
}

const FIXTURES: [Fixture; 4] = [
    Fixture {
        name: "Pico",
        ids: [
            "boards.pico.period",
            "boards.pico.pwm",
            "boards.pico.adc_uart",
            "boards.pico.watchdog",
            "boards.pico.transitions",
        ],
        board: BoardId::Rp2040,
        image: include_bytes!("../testdata/pico.elf"),
        led: 25,
        input: 3,
        pwm: 2,
        pwm_sample_hz: 125_000_000.0,
        watchdog_s: 0.008,
        channel_volts: 1.65,
        measurement_limit_s: 0.0065,
        reset_check_s: 0.009,
        frequency: 1_000_000.0,
        blink_ticks: 1000.0,
        pwm_high: 25.0,
        pwm_ticks: 100.0,
        adc_bins: 4096.0,
        vref: 3.3,
        clock_reference: "RP2040 §4.6, watchdog tick: 12 МГц / 12",
    },
    Fixture {
        name: "Blue Pill",
        ids: [
            "boards.stm.period",
            "boards.stm.pwm",
            "boards.stm.adc_uart",
            "boards.stm.watchdog",
            "boards.stm.systick",
        ],
        board: BoardId::Stm32f103c8,
        image: include_bytes!("../testdata/stm.elf"),
        led: 45,
        input: 7,
        pwm: 6,
        pwm_sample_hz: 8_000_000.0,
        watchdog_s: 0.032,
        channel_volts: 1.65,
        measurement_limit_s: 0.0065,
        reset_check_s: 0.034,
        frequency: 8_000_000.0 / 8.0,
        blink_ticks: 1000.0,
        pwm_high: 25.0,
        pwm_ticks: 100.0,
        adc_bins: 4096.0,
        vref: 3.3,
        clock_reference: "RM0008 TIM2: HSI 8 МГц, PSC=7, ARR=999",
    },
    Fixture {
        name: "Uno/Nano",
        ids: [
            "boards.avr.period",
            "boards.avr.pwm",
            "boards.avr.adc_uart",
            "boards.avr.watchdog",
            "boards.avr.transitions",
        ],
        board: BoardId::Atmega328p,
        image: include_bytes!("../testdata/avr.hex"),
        led: 13,
        input: 7,
        pwm: 11,
        pwm_sample_hz: 16_000_000.0 / 8.0,
        watchdog_s: 0.016,
        channel_volts: 2.5,
        measurement_limit_s: 0.0065,
        reset_check_s: 0.018,
        frequency: 16_000_000.0 / 64.0,
        blink_ticks: 250.0,
        pwm_high: 64.0,
        pwm_ticks: 256.0,
        adc_bins: 1024.0,
        vref: 5.0,
        clock_reference: "ATmega328P §15, CTC: 16 МГц, /64, OCR0A=249",
    },
    Fixture {
        name: "ESP32-C3",
        ids: [
            "boards.esp.period",
            "boards.esp.pwm",
            "boards.esp.adc_uart",
            "boards.esp.watchdog",
            "boards.esp.transitions",
        ],
        board: BoardId::Esp32C3,
        image: include_bytes!("../testdata/esp.elf"),
        led: 3,
        input: 10,
        pwm: 4,
        pwm_sample_hz: 80_000_000.0,
        watchdog_s: 0.008,
        channel_volts: 1.25,
        measurement_limit_s: 0.0065,
        reset_check_s: 0.009,
        frequency: 80_000_000.0 / 80.0,
        blink_ticks: 1000.0,
        pwm_high: 64.0,
        pwm_ticks: 256.0,
        adc_bins: 4096.0,
        vref: 1.1,
        clock_reference: "ESP32-C3 TRM TIMG: APB 80 МГц, /80, ALARM0=1000",
    },
];

// Шаг измерения, выбранный для стенда: 100 отсчётов на полупериод в 1 мс.
const SAMPLE_S: f64 = 0.00001;

struct Measurement {
    period: f64,
    duty: f64,
    adc: f64,
    reset_delay: f64,
    extra: bool,
}

fn measure(f: &Fixture) -> Result<Measurement, BoardFault> {
    let mut board = new_board(f.board);
    board.set_pin_input(f.input, true);
    board.set_adc_volts(0, f.channel_volts);
    board.load(f.image)?;
    let mut prior = None;
    let mut rising = Vec::new();
    let mut falling = 0;
    let samples = (f.measurement_limit_s / SAMPLE_S).round() as u32;
    for sample in 1..=samples {
        board.run_for(SAMPLE_S)?;
        let level = board.pin_output(f.led);
        if let (Some(old), Some(new)) = (prior, level)
            && old != new
        {
            if new {
                rising.push(sample);
            } else {
                falling += 1;
            }
        }
        prior = level;
    }
    let period = match rising.as_slice() {
        [first, next, tail @ ..] if falling >= 2 => tail
            .iter()
            .try_fold(*next, |previous, t| {
                (t.abs_diff(previous).abs_diff(next - first) <= 1).then_some(*t)
            })
            .map_or(f64::NAN, |_| f64::from(next - first) * SAMPLE_S),
        _ => f64::NAN,
    };
    let mut high = 0;
    let mut connected = true;
    for _ in 0..f.pwm_ticks as u32 {
        board.run_for(f.pwm_sample_hz.recip())?;
        match board.pin_output(f.pwm) {
            Some(true) => high += 1,
            Some(false) => {}
            None => connected = false,
        }
    }
    let reported = board.pwm_duty(f.pwm);
    let observed = f64::from(high) / f.pwm_ticks;
    let duty = reported
        .filter(|reported| connected && (reported - observed).abs() <= f.pwm_ticks.recip())
        .map_or(f64::NAN, |_| observed);
    let adc = match board.uart_take().as_slice() {
        [hi, lo] => f64::from(u16::from_be_bytes([*hi, *lo])),
        _ => f64::NAN,
    };
    let extra = if f.board == BoardId::Stm32f103c8 {
        board.pin_output(5) == Some(true) && board.pin_output(4) == Some(true)
    } else {
        rising.len() >= 3 && falling >= 3
    };
    let waveform_duration = f.pwm_ticks / f.pwm_sample_hz;
    let mut current = f.measurement_limit_s + waveform_duration;
    let mut reset_delay = f64::NAN;
    while current < f.reset_check_s {
        board.run_for(SAMPLE_S)?;
        current += SAMPLE_S;
        let bytes = board.uart_take();
        if bytes == b"R" && board.pin_output(f.led) == Some(true) && board.pwm_duty(f.pwm).is_none()
        {
            if let Some(first) = rising.first() {
                let started = f64::from(*first) * SAMPLE_S - f.blink_ticks / f.frequency;
                reset_delay = current - started;
            }
            break;
        }
        if !bytes.is_empty() {
            break;
        }
    }
    Ok(Measurement {
        period,
        duty,
        adc,
        reset_delay,
        extra,
    })
}

pub(crate) fn all() -> Vec<Check> {
    let mut checks = Vec::new();
    for f in &FIXTURES {
        let result = measure(f);
        let read = |by: fn(&Measurement) -> f64| result.as_ref().map_or(f64::NAN, by);
        let note = result.as_ref().err().map(ToString::to_string);
        let m = if f.board == BoardId::Atmega328p {
            "машинный образ AVRe из ручных кодировок"
        } else {
            "образ no_std Rust"
        };
        let (adc_ref, adc_formula) = if f.board == BoardId::Esp32C3 {
            (
                (1.25_f64 * 0.25 * 4095.0 / 1.1).floor(),
                "ESP32-C3 TRM §34.2.4: идеальный raw-код Vin × k × 4095 / Vref",
            )
        } else {
            (
                (f.channel_volts * f.adc_bins / f.vref).floor(),
                "идеальное прямое двоичное квантование: floor(Vin × 2^N / Vref)",
            )
        };
        let values = [
            (
                "период мигания",
                read(|r| r.period),
                2.0 * f.blink_ticks / f.frequency,
                "с",
                Tolerance::Absolute(SAMPLE_S),
                f.clock_reference,
                "два переключения составляют период",
            ),
            (
                "скважность ШИМ",
                read(|r| r.duty),
                f.pwm_high / f.pwm_ticks,
                "доля",
                Tolerance::Absolute(f.pwm_ticks.recip()),
                "число высоких отсчётов и период заданы в исходнике прошивки",
                "измерено по уровню вывода, допуск — один отсчёт",
            ),
            (
                "код АЦП через UART",
                read(|r| r.adc),
                adc_ref,
                "код",
                Tolerance::Absolute(0.0),
                adc_formula,
                "эталон идеализирован, калибровка платы не измерялась",
            ),
            (
                "сброс по сторожевому таймеру",
                read(|r| r.reset_delay),
                f.watchdog_s,
                "с",
                Tolerance::Absolute(SAMPLE_S * 4.0),
                "тайм-аут из частоты, предделителя и порога, указанных в исходнике прошивки",
                "от первого фронта минус полупериод до байта R; допуск включает UART и два края окна выборки",
            ),
            (
                if f.board == BoardId::Stm32f103c8 {
                    "исключение SysTick и вытеснение TIM2"
                } else {
                    "повторные прерывания таймера"
                },
                read(|r| f64::from(u8::from(r.extra))),
                1.0,
                "",
                Tolerance::Absolute(0.0),
                "обработчик прерывания оставляет наблюдаемый уровень/фронты GPIO",
                "проверка маршрута прерывания и возврата в программу",
            ),
        ];
        for ((title, simulated, reference, unit, tolerance, reference_method, detail), id) in
            values.into_iter().zip(f.ids)
        {
            checks.push(Check {
                id,
                title: format!("{}: {title}", f.name),
                simulated,
                reference,
                unit,
                method: m.into(),
                reference_method: reference_method.into(),
                tolerance,
                note: note.clone().unwrap_or_else(|| detail.into()),
            });
        }
    }
    checks
}
