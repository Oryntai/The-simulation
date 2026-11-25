//! Эталоны штатных SDK-сборок, снятые на QEMU Espressif. Эмулятор платы в опыте не участвует.

use std::{env, path::Path, process::Command};

const HELLO: &str = include_str!("../firmware-tests/esp_sdk/reference/hello_world.trace");
const BLINK: &str = include_str!("../firmware-tests/esp_sdk/reference/blink.trace");
const MMIO_HEADER: &str = "--- GPIO MMIO: байты UART, адрес, значение, ширина ---\n";

// ESP32-C3 TRM v1.4, регистры 5.3, 5.4, 5.6; база из ESP-IDF v6.1.1,
// components/soc/esp32c3/register/soc/{reg_base,gpio_reg}.h.
const GPIO_SET: &str = "0x60004008";
const GPIO_CLEAR: &str = "0x6000400c";
const GPIO_ENABLE: &str = "0x60004024";
// Blink/Blink.ino и даташит ESP32-C3 v2.5, таблица 2-7.
const PIN: u32 = 3;
const PIN_MASK: u32 = 1 << PIN;
// Два цикла Blink в окне захвата capture.py.
const EDGES: usize = 4;
// Blink/Blink.ino и esp32c3-libs 4.0.0/sdkconfig: CONFIG_FREERTOS_HZ=1000.
const HALF_PERIOD_MS: u64 = 1000;
const TICK_HZ: u64 = 1000;
// Arduino ESP32 4.0.0, cores/esp32/main.cpp: vTaskDelay(5) в yieldIfNecessary.
const UNICORE_YIELD_TICKS: u64 = 5;

fn hello_moments(trace: &str) -> Result<[u64; 3], &'static str> {
    if !trace.ends_with('\n') || !trace.lines().any(|line| line.contains("(POWERON)")) {
        return Err("нет полной трассы холодного старта");
    }
    let mut lines = trace.lines();
    let mut previous = 0;
    let mut moments = [0; 3];
    // ESP-IDF v6.1.1: partitions_singleapp.csv и адрес factory из flasher_args.json.
    for (moment, expected) in moments.iter_mut().zip([
        "boot: ESP-IDF v6.1.1 2nd stage bootloader",
        "boot: Loaded app from partition at offset 0x10000",
        "main_task: Calling app_main()",
    ]) {
        loop {
            let line = lines.next().ok_or("потеряна стадия загрузки")?;
            let Some(log) = line.strip_prefix("I (") else {
                continue;
            };
            let (ms, message) = log.split_once(") ").ok_or("формат времени лога IDF")?;
            let ms = ms
                .parse::<u64>()
                .map_err(|_| "время лога IDF не в целых мс")?;
            if ms < previous {
                return Err("время лога IDF пошло назад");
            }
            previous = ms;
            if message == expected {
                *moment = ms;
                break;
            }
        }
    }
    if lines.next() != Some("Hello world!") {
        return Err("нет UART после вызова app_main");
    }
    // ESP-IDF v6.1.1, examples/get-started/hello_world/main/hello_world_main.c.
    let mut countdown = lines.skip(2);
    for second in (0..=10).rev() {
        if countdown.next() != Some(format!("Restarting in {second} seconds...").as_str()) {
            return Err("нарушен обратный отсчёт Hello World");
        }
    }
    if countdown.next() != Some("Restarting now.") || countdown.next().is_some() {
        return Err("нет штатного окончания Hello World");
    }
    Ok(moments)
}

#[derive(Clone, Copy)]
struct Edge {
    level: u8,
    tick: u64,
    before_us: u64,
    after_us: u64,
    uart_start: usize,
    uart_end: usize,
}

fn blink_edges(trace: &str) -> Result<Vec<Edge>, &'static str> {
    let (uart, mmio) = trace
        .split_once(MMIO_HEADER)
        .ok_or("нет секции GPIO MMIO")?;
    if !uart.lines().any(|line| line.contains("(POWERON)")) {
        return Err("нет холодного старта Blink");
    }
    let mut edges: Vec<Edge> = Vec::new();
    let mut offset = 0;
    for line in uart.split_inclusive('\n') {
        let start = offset;
        offset += line.len();
        let Some(report) = line.strip_prefix("Blink GPIO ") else {
            if !edges.is_empty() {
                return Err("лишний вывод во время Blink");
            }
            continue;
        };
        let fields: Vec<_> = report.split_ascii_whitespace().collect();
        let [pin, level, tick, before, after] = fields.as_slice() else {
            return Err("неполная метка Blink");
        };
        let level = level.parse().map_err(|_| "нечисловой уровень Blink")?;
        if pin.parse::<u32>() != Ok(PIN) || level != u8::from(edges.len().is_multiple_of(2)) {
            return Err("неверный вывод или порядок уровней Blink");
        }
        let edge = Edge {
            level,
            tick: tick.parse().map_err(|_| "некорректный такт FreeRTOS")?,
            before_us: before
                .parse()
                .map_err(|_| "некорректное время до digitalWrite")?,
            after_us: after
                .parse()
                .map_err(|_| "некорректное время после digitalWrite")?,
            uart_start: start,
            uart_end: offset,
        };
        if edge.after_us < edge.before_us || edge.after_us - edge.before_us >= 1_000_000 / TICK_HZ {
            return Err("неверный интервал digitalWrite");
        }
        if let Some(previous) = edges.last() {
            if edge.before_us <= previous.after_us {
                return Err("интервалы digitalWrite пересекаются");
            }
            // delay → vTaskDelay в cores/esp32/esp32-hal-misc.c. Первый вызов между
            // итерациями loop после порога 2000 мс добавляет пять тактов из main.cpp.
            let mut ticks = HALF_PERIOD_MS * TICK_HZ / 1000;
            if edges.len() == 2 {
                ticks += UNICORE_YIELD_TICKS;
            }
            if edge.tick.checked_sub(previous.tick) != Some(ticks) {
                return Err("полупериод Blink не равен ожидаемому числу тактов FreeRTOS");
            }
            // Отсчёты esp_timer_get_time внутри такта. Допуск в один такт покрывает фазу
            // между отсчётом таймера и границей такта FreeRTOS при сравнении двух шкал.
            let tick_us = 1_000_000 / TICK_HZ;
            let elapsed_us = edge.before_us - previous.before_us;
            if elapsed_us.abs_diff(ticks * tick_us) >= tick_us {
                return Err("время SYSTIMER расходится с тактами FreeRTOS");
            }
        }
        edges.push(edge);
    }
    if edges.len() != EDGES || !mmio.ends_with('\n') {
        return Err("неполная трасса Blink");
    }

    let mut writes = mmio.lines();
    let enable_offset = gpio_write(writes.next(), GPIO_ENABLE)?;
    let mut previous_end = 0;
    for edge in &edges {
        let register = if edge.level == 1 {
            GPIO_SET
        } else {
            GPIO_CLEAR
        };
        let write_offset = gpio_write(writes.next(), register)?;
        if write_offset < previous_end
            || write_offset < enable_offset
            || write_offset > edge.uart_start
        {
            return Err("запись MMIO вне окна своей метки UART");
        }
        previous_end = edge.uart_end;
    }
    if writes.next().is_some() {
        return Err("лишняя запись GPIO MMIO");
    }
    Ok(edges)
}

fn gpio_write(row: Option<&str>, expected_register: &str) -> Result<usize, &'static str> {
    let fields: Vec<_> = row
        .ok_or("пропущена запись GPIO MMIO")?
        .split_ascii_whitespace()
        .collect();
    let [offset, register, value, width] = fields.as_slice() else {
        return Err("неполная запись GPIO MMIO");
    };
    let value = value
        .strip_prefix("0x")
        .and_then(|hex| u32::from_str_radix(hex, 16).ok());
    // В TRM v1.4, гл. 5, GPIO-регистры 32-битные, значащий бит один на вывод.
    if *register != expected_register || value != Some(PIN_MASK) || *width != "4" {
        return Err("адрес, маска или ширина записи GPIO MMIO не совпали");
    }
    offset.parse().map_err(|_| "неверное смещение UART")
}

#[test]
fn esp_idf_reference_reaches_app_and_finishes_countdown() {
    let [boot, loaded, app] = hello_moments(HELLO).unwrap();
    assert!(boot < loaded && loaded < app);
}

#[test]
fn arduino_reference_matches_gpio_transactions_and_rtos_period() {
    assert_eq!(blink_edges(BLINK).unwrap().len(), EDGES);
}

#[test]
fn esp_idf_reference_rejects_missing_app_wrong_countdown_and_invalid_time() {
    for (original, damaged, error) in [
        ("Hello world!\n", "", "нет UART после вызова app_main"),
        (
            "Restarting in 9",
            "Restarting in 8",
            "нарушен обратный отсчёт Hello World",
        ),
        ("I (", "I (NaN", "время лога IDF не в целых мс"),
    ] {
        assert_eq!(
            hello_moments(&HELLO.replacen(original, damaged, 1)),
            Err(error)
        );
    }
}

#[test]
fn arduino_reference_rejects_bad_mmio_missing_marker_and_invalid_time() {
    let invalid_register = BLINK.replacen(GPIO_SET, GPIO_CLEAR, 1);
    assert_eq!(
        blink_edges(&invalid_register).err(),
        Some("адрес, маска или ширина записи GPIO MMIO не совпали")
    );
    let missing_marker: String = BLINK
        .split_inclusive('\n')
        .filter(|line| !line.starts_with("Blink GPIO 3 0"))
        .collect();
    assert_eq!(
        blink_edges(&missing_marker).err(),
        Some("неверный вывод или порядок уровней Blink")
    );
    let edge = blink_edges(BLINK).unwrap()[0];
    let invalid_time = BLINK.replacen(&format!(" {} ", edge.before_us), " inf ", 1);
    assert_eq!(
        blink_edges(&invalid_time).err(),
        Some("некорректное время до digitalWrite")
    );
    let wrong_tick = BLINK.replacen(
        &format!(" {} {} ", edge.tick, edge.before_us),
        &format!(" {} {} ", edge.tick + 1, edge.before_us),
        1,
    );
    assert_eq!(
        blink_edges(&wrong_tick).err(),
        Some("полупериод Blink не равен ожидаемому числу тактов FreeRTOS")
    );
}

#[test]
#[ignore = "нужны ESP-IDF v6.1.1, Arduino ESP32 4.0.0, arduino-cli и QEMU Espressif 9.2.2"]
fn installed_sdks_reproduce_qemu_references() {
    let script = Path::new(env!("CARGO_MANIFEST_DIR")).join("firmware-tests/esp_sdk/capture.sh");
    let temp_root = env::var_os("SIM_ESP_SDK_TMPDIR").map_or_else(env::temp_dir, Into::into);
    for (case, reference) in [("hello_world", HELLO), ("blink", BLINK)] {
        let output = Command::new("bash")
            .args([script.as_os_str(), case.as_ref()])
            .env("SIM_ESP_SDK_TMPDIR", &temp_root)
            .output()
            .expect("запуск capture.sh");
        assert!(
            output.status.success(),
            "{case}: {}\n{}",
            output.status,
            String::from_utf8_lossy(&output.stderr)
        );
        let actual = std::str::from_utf8(&output.stdout).expect("трасса UTF-8");
        assert_eq!(
            actual, reference,
            "расхождение новой трассы {case} с эталоном"
        );
    }
}
