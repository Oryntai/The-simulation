//! Стенд штатных образов ESP32-C3 на холодном старте из ROM.

use sim_boards::{
    BoardFault, BoardId, BoardInput, BoardOutput, BoardSim, EspBootOptions, EspResetProfile,
    I2cRequest, InputEvent, InputOutcome, OutputEvent, SystimerPhase, TransferStatus,
    esp_flash::EspFlashReadback, esp_image, new_board,
};

const RESET_PROFILE: &str =
    include_str!("../firmware-tests/esp_sdk/reference/esp32c3-qemu-reset.toml");
const IO_PROFILE: &str = include_str!("data/esp_sdk_acceptance/io-reset.toml");
const BLINK: &[u8] = include_bytes!("data/esp_sdk_acceptance/blink.bin");
const HELLO: &[u8] = include_bytes!("data/esp_sdk_acceptance/hello.bin");
const HELLO_QEMU: &str = include_str!("../firmware-tests/esp_sdk/reference/hello_world.trace");
const EFUSE: &[u8] = include_bytes!("data/esp_sdk_acceptance/qemu-efuse-readback.bin");
const PERIPHERALS: &[u8] = include_bytes!("data/esp_peripherals/flash.bin");
// ESP-IDF soc/esp32c3/register/soc/efuse_reg.h, EFUSE_RD_WR_DIS..=EFUSE_RD_SYS_PART2_DATA7.
const EFUSE_BYTES: usize = 0x17c - 0x2c;
const TABLE: u32 = esp_image::DEFAULT_PARTITION_TABLE_OFFSET;
// По README стенда 2.21. Вход GPIO4 не используется как strap.
const GPIO_OUT: u32 = 3;
const GPIO_IN: u32 = 10;
const WAKE: u32 = 4;

fn fixture(key: &str) -> Vec<u8> {
    let path = std::env::var_os(key).unwrap_or_else(|| panic!("NotAnalyzed: задайте {key}"));
    std::fs::read(&path).unwrap_or_else(|error| panic!("чтение {key} ({path:?}): {error}"))
}

fn efuse() -> Vec<u32> {
    let bytes = EFUSE;
    assert_eq!(bytes.len(), EFUSE_BYTES);
    bytes
        .chunks_exact(size_of::<u32>())
        .map(|word| u32::from_le_bytes(word.try_into().unwrap()))
        .collect()
}

fn load(
    board: &mut dyn BoardSim,
    flash: &[u8],
    rom: &[u8],
    efuse: &[u32],
    profiled: bool,
) -> Result<(), BoardFault> {
    let capacity = esp_image::parse_flash(flash, TABLE)?
        .bootloader()
        .header
        .flash_size;
    // Espressif QEMU 9.2.2, hw/riscv/esp32c3.c и hw/block/m25p80.c:
    // по ёмкости выбираются w25x16 либо gd25q32.
    let jedec_id = match capacity {
        0x20_0000 => [0xef, 0x30, 0x15],
        0x40_0000 => [0xc8, 0x40, 0x16],
        _ => panic!("ёмкость {capacity} вне стенда"),
    };
    let options = EspBootOptions {
        systimer_phase: Some(SystimerPhase::FirstAfterTwoXtalCycles),
        flash_readback: Some(EspFlashReadback {
            jedec_id,
            status1: 0,
            status2: None,
        }),
    };
    // Снимок GPIO_STRAP_REG из монитора QEMU: 0x08, как в esp_sdk_boot.
    let straps = [false, false, true];
    if profiled {
        board.load_esp_flash_with_profile(
            flash,
            TABLE,
            rom,
            efuse,
            straps,
            options
                .with_reset_profile(
                    EspResetProfile::parse(&format!("{RESET_PROFILE}\n{IO_PROFILE}")).unwrap(),
                )
                .with_virtual_rng_seed(sim_signal::fnv1a(flash.iter().copied())),
        )
    } else {
        board.load_esp_flash_with_options(flash, TABLE, rom, efuse, straps, options)
    }
}

fn input(board: &mut dyn BoardSim, input: BoardInput) -> InputOutcome {
    board
        .apply(InputEvent {
            cycle: board.cycles(),
            input,
        })
        .unwrap()
}

fn gpio(board: &mut dyn BoardSim, pin: u32, high: bool) {
    assert!(matches!(
        input(board, BoardInput::Gpio { pin, high }),
        InputOutcome::Applied | InputOutcome::Unchanged
    ));
}

fn advance(board: &mut dyn BoardSim, deadline: u128, uart: &mut Vec<u8>) {
    let result = board.run_until_quarter_ns(deadline);
    let events = board.take_events();
    uart.extend(events.iter().filter_map(|event| match event.output {
        BoardOutput::Uart { port: 0, byte } => Some(byte),
        _ => None,
    }));
    if let Err(fault) = result {
        panic!(
            "{:?}, такт {}, {fault}\nUART0:\n{}",
            board.esp_boot_position(),
            board.cycles(),
            String::from_utf8_lossy(uart)
        );
    }
    assert!(
        result
            .unwrap()
            .is_some_and(|state| !state.bus_request_pending)
    );
}

fn qns(board: &dyn BoardSim, cycle: u64) -> u128 {
    board.cycle_time_quarter_ns(cycle).unwrap().unwrap()
}

// Шкала BoardSim::run_until_quarter_ns. Секунда используется как бюджет стенда.
const SECOND: u128 = 4_000_000_000;
const MS: u128 = SECOND / 1000;

#[test]
#[ignore = "нужен ROM Espressif из SIM_ESP_ACCEPTANCE_ROM"]
fn hello_world() {
    let mut board = new_board(BoardId::Esp32C3);
    load(
        &mut *board,
        HELLO,
        &fixture("SIM_ESP_ACCEPTANCE_ROM"),
        &efuse(),
        true,
    )
    .unwrap();
    // Трасса Т2.10 и nm штатного ELF: первый вход в app_main на следующей
    // инструкции. Это контрольная точка входа, а не бюджет загрузки.
    assert_eq!(board.run_cycles(23_126_661).unwrap().cycle, 23_126_661);
    let position = board.esp_boot_position().unwrap();
    assert_eq!((position.pc, position.step), (0x4200_6bda, 23_126_661));
    let mut uart = Vec::new();
    let reference = HELLO_QEMU
        .lines()
        .find(|line| *line == "Hello world!")
        .unwrap();
    let line = format!("{reference}\r\n");
    // Верхняя граница стенда, достаточная для последовательного вывода UART
    // и холодной загрузки. Успех задаётся обязательной строкой UART0.
    let first_poll = qns(&*board, board.cycles()).div_ceil(MS);
    for ms in first_poll..=1000 {
        advance(&mut *board, ms * MS, &mut uart);
        if uart
            .split_inclusive(|&byte| byte == b'\n')
            .any(|part| part == line.as_bytes())
        {
            assert_eq!(board.reset_cause(), Some(sim_boards::ResetCause::PowerOn));
            assert!(board.capabilities().blocks_hardware_proof);
            assert!(String::from_utf8_lossy(&uart).contains("Calling app_main()\r\n"));
            assert!(!board.esp_reset_value_uses().is_empty());
            assert!(board.esp_reset_value_uses().iter().all(|used| {
                used.entry.kind == sim_boards::EspResetValueKind::Qemu && used.reads > 0
            }));
            eprintln!("UART0: {reference}, отметка опроса {ms} мс");
            return;
        }
    }
    panic!(
        "нет строки {reference:?}\n{}",
        String::from_utf8_lossy(&uart)
    );
}

#[test]
#[ignore = "нужен ROM Espressif из SIM_ESP_ACCEPTANCE_ROM"]
fn отвергаются_раздел_efuse_и_чтение_mmio_без_профиля() {
    let rom = fixture("SIM_ESP_ACCEPTANCE_ROM");
    let fuse = efuse();
    let mut board = new_board(BoardId::Esp32C3);
    let mut flash = HELLO.to_vec();
    // ESP-IDF gen_esp32part.py: magic первой записи таблицы. Та же
    // мутация в capture.py даёт Failed to verify partition table в QEMU.
    flash[TABLE as usize] ^= 1;
    let bad_table = board
        .load_esp_flash(&flash, TABLE, &rom, &fuse, [false, false, true])
        .unwrap_err();
    assert!(matches!(&bad_table, BoardFault::BadImage(text) if text.contains("сигнатура")));
    assert_eq!(board.run_cycles(1), Err(bad_table));
    assert_eq!(board.cycles(), 0);
    let mut encrypted = fuse.clone();
    // TRM §23.3, efuse_reg.h: нечётное число SPI_BOOT_CRYPT_CNT[20:18].
    encrypted[(0x34 - 0x2c) / size_of::<u32>()] ^= 1 << 18;
    let bad_fuse = load(&mut *board, HELLO, &rom, &encrypted, true).unwrap_err();
    assert!(
        matches!(&bad_fuse, BoardFault::Unsupported(text) if text.contains("шифрованием flash"))
    );
    assert_eq!(board.run_cycles(1), Err(bad_fuse));
    assert_eq!(board.cycles(), 0);
    load(&mut *board, HELLO, &rom, &fuse, false).unwrap();
    // Т2.10, esp_sdk_boot: первое чтение ANA_CONFIG_REG загрузчиком v6.1.1.
    let err = BoardFault::UnsupportedRegister {
        pc: 0x403c_ca60,
        addr: 0x6000_e044,
        size: 4,
        write: false,
        block: "RTC REGI2C: состояние холодного сброса не задокументировано",
    };
    // Внешний лимит исполнения, не допуск измерения.
    assert_eq!(board.run_cycles(1_000_000), Err(err.clone()));
    let stopped = board.esp_boot_position().unwrap();
    assert_eq!(board.run_cycles(1), Err(err));
    assert_eq!(board.esp_boot_position(), Some(stopped));
    assert!(board.esp_reset_value_uses().is_empty());
    assert!(
        !board
            .uart_take()
            .windows(b"Hello world!".len())
            .any(|bytes| bytes == b"Hello world!")
    );
}

const BLINK_QEMU: &str = include_str!("data/esp_sdk_acceptance/blink.trace");
const PERIPHERALS_QEMU: &str = include_str!("data/esp_peripherals/qemu.trace");

fn blink_rows(trace: &str) -> Vec<(bool, u128, u128, u128)> {
    trace
        .lines()
        .filter_map(|line| line.strip_prefix("Blink GPIO "))
        .map(|line| {
            let fields: Vec<u128> = line
                .split_ascii_whitespace()
                .map(|field| field.parse().unwrap())
                .collect();
            let [pin, level @ (0 | 1), tick, before, after] = fields[..] else {
                panic!("неверная строка Blink: {line}")
            };
            assert_eq!(pin, u128::from(GPIO_OUT));
            assert!(before <= after);
            (level == 1, tick, before, after)
        })
        .collect()
}

#[test]
#[ignore = "нужен ROM Espressif из SIM_ESP_ACCEPTANCE_ROM"]
fn blink() {
    let mut board = new_board(BoardId::Esp32C3);
    load(
        &mut *board,
        BLINK,
        &fixture("SIM_ESP_ACCEPTANCE_ROM"),
        &efuse(),
        true,
    )
    .unwrap();
    let mut uart = Vec::new();
    let mut edges = Vec::new();
    let mut last = false; // Окно измерения начинается с первого высокого уровня.
    // Blink.ino: четыре команды, из которых первая стартует после загрузки,
    // три задержки по 1000 мс. Одна секунда сверху — бюджет стенда.
    for poll in 1..=4000 {
        advance(&mut *board, poll * MS, &mut uart);
        let level = board.pin_output(GPIO_OUT);
        if !edges.is_empty() {
            assert!(level.is_some());
        }
        if let Some(level) = level
            && level != last
        {
            edges.push((level, qns(&*board, board.cycles())));
            last = level;
        }
        if uart
            .split(|&byte| byte == b'\n')
            .filter(|line| line.starts_with(b"Blink GPIO ") && line.ends_with(b"\r"))
            .count()
            == 4
        {
            break;
        }
    }

    let report = blink_rows(std::str::from_utf8(&uart).unwrap());
    let reference = blink_rows(BLINK_QEMU);
    let (_, qemu_mmio) = BLINK_QEMU
        .split_once("--- GPIO MMIO: байты UART, адрес, значение, ширина ---\n")
        .unwrap();
    let qemu_addresses: Vec<_> = qemu_mmio
        .lines()
        .map(|line| {
            let fields: Vec<_> = line.split_ascii_whitespace().collect();
            let [offset, address, mask, width] = fields[..] else {
                panic!("неверная строка GPIO MMIO: {line}");
            };
            assert!(offset.parse::<u32>().is_ok());
            assert_eq!(
                u32::from_str_radix(mask.strip_prefix("0x").unwrap(), 16),
                Ok(1 << GPIO_OUT)
            );
            assert_eq!(width.parse::<usize>(), Ok(size_of::<u32>()));
            address
        })
        .collect();
    // TRM рег. 5.6, 5.3, 5.4: включение выхода, установка и очистка.
    assert_eq!(
        qemu_addresses,
        [
            "0x60004024",
            "0x60004008",
            "0x6000400c",
            "0x60004008",
            "0x6000400c"
        ]
    );
    assert_eq!(
        edges.len(),
        4,
        "{edges:?}, {}",
        String::from_utf8_lossy(&uart)
    );
    assert_eq!(report.len(), 4);
    assert_eq!(reference.len(), report.len());
    for (i, (&(level, _), row)) in edges.iter().zip(&report).enumerate() {
        assert_eq!(level, i % 2 == 0);
        assert_eq!(row.0, level);
        assert_eq!(reference[i].0, level);
    }
    // esp32c3-libs/4.0.0/sdkconfig: 1 кГц. Arduino main.cpp:
    // yieldIfNecessary по порогу 2000 мс добавляет vTaskDelay(5).
    // Квант фазы FreeRTOS — 1 мс, опрос выводов добавляет до 1 мс.
    // esp_timer_get_time в обеих машинах возвращает целые микросекунды.
    const US: u128 = MS / 1000;
    for (i, expected_ticks) in [1000, 1005, 1000].into_iter().enumerate() {
        let dt = edges[i + 1].1 - edges[i].1;
        let dt_guest = (report[i + 1].2 - report[i].2) * US;
        let dt_qemu = (reference[i + 1].2 - reference[i].2) * US;
        assert_eq!(report[i + 1].1 - report[i].1, expected_ticks);
        assert_eq!(reference[i + 1].1 - reference[i].1, expected_ticks);
        assert!(dt_guest.abs_diff(expected_ticks * MS) <= MS + US);
        assert!(
            dt.abs_diff(dt_guest)
                <= MS + (report[i].3 - report[i].2 + report[i + 1].3 - report[i + 1].2 + 1) * US
        );
        assert!(dt_guest.abs_diff(dt_qemu) <= 2 * (MS + US));
        eprintln!(
            "Blink: {} тиков, вывод {} мкс, SDK {} мкс, QEMU {} мкс",
            expected_ticks,
            dt / US,
            dt_guest / US,
            dt_qemu / US
        );
    }
    assert!(board.capabilities().blocks_hardware_proof);
}

// ESP32-C3 TRM §§6.2, 26.4, 28.4, 32.3 и программа стенда 2.21:
// номинальный XTAL 40 МГц, CPU 160 МГц, 8N1, I²C 100 кГц, PWM 4 кГц.
const XTAL_HZ: u128 = 40_000_000;
const CPU_HZ: u128 = 160_000_000;
const CPU_QNS: u128 = SECOND / CPU_HZ;
const PWM: u32 = 5;
const FRAME: [u8; 4] = [0x55, 0xa6, 0x00, 0xff];
const I2C_SCL: u128 = SECOND / 100_000;
// 200 фронтов XTAL — 5 мкс. Это шаг наблюдения цифровых выводов стенда.
const POLL: u128 = 200 * (SECOND / XTAL_HZ);

fn stop_without_i2c_reply(board: &dyn BoardSim, request: &I2cRequest) {
    let mut fork = board.clone_state();
    // i2c_master_probe запрашивает ожидание 20 мс. i2c_ll_calculate_timeout_us_to_reg_val
    // кодирует bit_length(40 МГц * 20 мс) = 20; по TRM рег. I2C_TO это 2^20 тактов.
    let requested = XTAL_HZ * (20 * MS) / SECOND;
    let exponent = u128::BITS - requested.leading_zeros();
    let timeout = (1u64 << exponent) * (CPU_HZ / XTAL_HZ) as u64;
    let deadline = request.address_ack_cycle + timeout;
    assert_eq!(
        fork.run_cycles(deadline - 1 - fork.cycles()).unwrap().cycle,
        deadline - 1
    );
    let is_i2c = |event: &OutputEvent| match event.output {
        BoardOutput::Uart { port: 0, .. } => false,
        BoardOutput::I2c(_) | BoardOutput::I2cDone { .. } => true,
        _ => panic!("неожиданное событие в ветви I²C: {event:?}"),
    };
    assert!(!fork.take_events().iter().any(is_i2c));
    assert_eq!(fork.run_cycles(1).unwrap().cycle, deadline);
    assert_eq!(
        fork.take_events()
            .into_iter()
            .filter(is_i2c)
            .collect::<Vec<_>>(),
        [OutputEvent {
            cycle: deadline,
            output: BoardOutput::I2cDone {
                bus: request.bus,
                id: request.id,
                address: request.address,
                status: TransferStatus::NoResponse,
                bytes: vec![],
            },
        }]
    );
    assert!(
        fork.apply(InputEvent {
            cycle: deadline,
            input: BoardInput::I2cReply {
                bus: request.bus,
                id: request.id,
                status: TransferStatus::Complete,
                bytes: vec![]
            },
        })
        .is_err()
    );
    eprintln!(
        "I²C без ответа: тайм-аут через {} нс после ACK",
        u128::from(timeout) * CPU_QNS / 4
    );
}

fn check_uart(times: &[(u8, u128)]) {
    assert_eq!(
        times.iter().map(|&(byte, _)| byte).collect::<Vec<_>>(),
        FRAME
    );
    assert!(PERIPHERALS_QEMU.contains("UART1 host TX=55 a6 00 ff RX=55 a6 00 ff"));
    // uart_ll_set_baudrate: floor(16*40 МГц/115200)=5555. Один кадр
    // 8N1 занимает 10 битов. Сравнение на удвоенной шкале четвертей нс
    // сохраняет дробный результат. Метка кадра округлена к такту CPU.
    let divider = 16 * XTAL_HZ / 115_200;
    assert!(PERIPHERALS_QEMU.contains(&format!("divisor_x16={divider}")));
    let frame_x2 = 2 * 10 * divider * (SECOND / XTAL_HZ) / 16;
    for pair in times.windows(2) {
        let dt = pair[1].1 - pair[0].1;
        assert!(
            (2 * dt).abs_diff(frame_x2) <= 2 * CPU_QNS,
            "UART1: {dt} четвертей нс против {frame_x2}/2"
        );
        eprintln!("UART1: {:.3} нс между кадрами", dt as f64 / 4.0);
    }
}

fn check_pwm(edges: &[(bool, u128)], high_counts: u128) {
    // TRM §32.3, делитель 2500/256 от XTAL, таймер 10 бит.
    let period = (SECOND / XTAL_HZ) * 2500 * (1 << 10) / 256;
    let high = period * high_counts / (1 << 10);
    assert!(
        edges.len() >= 6,
        "недостаточно фронтов для duty={high_counts}: {edges:?}"
    );
    for triple in edges.windows(3).skip(1) {
        let [(first, a), (middle, b), (last, c)] = triple else {
            unreachable!()
        };
        assert_eq!(first, last);
        assert_ne!(first, middle);
        assert!((c - a).abs_diff(period) <= POLL + CPU_QNS);
        let expected = if *first { high } else { period - high };
        assert!((b - a).abs_diff(expected) <= POLL + CPU_QNS);
    }
    eprintln!(
        "LEDC duty={high_counts}/1024: период {} мкс, высокий уровень {:.1} мкс, {} фронтов",
        period / (MS / 1000),
        high as f64 / (MS / 1000) as f64,
        edges.len()
    );
}

fn check_i2c(requests: &[(u64, I2cRequest)], done: &[OutputEvent]) {
    assert_eq!(requests.len(), 3);
    assert_eq!(done.len(), 3);
    for ((cycle, request), (index, event)) in requests.iter().zip(done.iter().enumerate()) {
        let (address, write, bytes, status) = match index {
            0 => (0x50, &[][..], &[][..], TransferStatus::Complete),
            1 => (0x51, &[][..], &[][..], TransferStatus::Nack),
            2 => (
                0x50,
                &[0x10][..],
                &[0x5a, 0xa5][..],
                TransferStatus::Complete,
            ),
            _ => unreachable!(),
        };
        assert_eq!(
            (request.address, &request.write[..], request.read_len),
            (address, write, bytes.len())
        );
        assert_eq!(request.repeated_start, index == 2);
        // Пять интервалов start/stop по half_cycle для combined (START,
        // repeated START, STOP), три для одиночной пробы. Девять SCL/кадр.
        let scl = (I2C_SCL / CPU_QNS) as u64;
        let frames = if index == 2 { 5 } else { 1 };
        let half_cycles = if index == 2 { 5 } else { 3 };
        assert_eq!(request.cycles_per_bit, scl);
        assert_eq!(request.address_ack_cycle - cycle, scl / 2 + 9 * scl);
        assert_eq!(
            request.complete_cycle - cycle,
            half_cycles * (scl / 2) + frames * 9 * scl
        );
        assert_eq!(
            *event,
            OutputEvent {
                cycle: if status == TransferStatus::Nack {
                    request.address_ack_cycle
                } else {
                    request.complete_cycle
                },
                output: BoardOutput::I2cDone {
                    bus: 0,
                    id: request.id,
                    address,
                    status,
                    bytes: bytes.to_vec(),
                },
            }
        );
        eprintln!(
            "I²C {address:#04x}: ACK-слот {} мкс, событие {} мкс",
            u128::from(request.address_ack_cycle - cycle) * CPU_QNS / (MS / 1000),
            u128::from(event.cycle - cycle) * CPU_QNS / (MS / 1000)
        );
    }
}

#[test]
#[ignore = "нужен ROM Espressif из SIM_ESP_ACCEPTANCE_ROM"]
fn штатная_периферия() {
    let mut board = new_board(BoardId::Esp32C3);
    load(
        &mut *board,
        PERIPHERALS,
        &fixture("SIM_ESP_ACCEPTANCE_ROM"),
        &efuse(),
        true,
    )
    .unwrap();
    gpio(&mut *board, WAKE, true);
    gpio(&mut *board, GPIO_IN, false);
    let mut uart = Vec::new();
    let mut frames = Vec::new();
    let mut requests = Vec::new();
    let mut done = Vec::new();
    let mut gpio_edges = Vec::new();
    let mut pwm_edges = [Vec::new(), Vec::new()];
    let mut last_gpio = false;
    let mut last_pwm = false;
    let mut pwm_stage = None;
    let mut pwm_stopped = false;
    let mut barrier = None;
    let mut last_sleep = None;
    for deadline in (POLL..=SECOND).step_by(POLL as usize) {
        loop {
            let result = board.run_until_quarter_ns(deadline);
            let (events, progress) = (board.take_events(), result);
            let mut echoed = false;
            for event in events {
                match event.output {
                    BoardOutput::Uart { port: 0, byte } => uart.push(byte),
                    BoardOutput::Uart { port: 1, byte } => {
                        frames.push((byte, qns(&*board, event.cycle)));
                        echoed = frames.len() == FRAME.len();
                    }
                    BoardOutput::I2c(request) => {
                        if requests.is_empty() {
                            stop_without_i2c_reply(&*board, &request);
                        }
                        let (status, bytes) = match (request.address, request.read_len) {
                            (0x50, 0) => (TransferStatus::Complete, vec![]),
                            (0x50, 2) => (TransferStatus::Complete, vec![0x5a, 0xa5]),
                            (0x51, 0) => (TransferStatus::Nack, vec![]),
                            _ => panic!("неожиданный запрос {request:?}"),
                        };
                        assert_eq!(
                            input(
                                &mut *board,
                                BoardInput::I2cReply {
                                    bus: request.bus,
                                    id: request.id,
                                    status,
                                    bytes
                                }
                            ),
                            InputOutcome::Applied
                        );
                        requests.push((event.cycle, request));
                    }
                    BoardOutput::I2cDone { .. } => done.push(event),
                    BoardOutput::Reset(cause) => panic!("лишний сброс: {cause:?}"),
                    _ => {}
                }
            }
            if echoed {
                assert_eq!(
                    input(
                        &mut *board,
                        BoardInput::Uart {
                            port: 1,
                            bytes: FRAME.to_vec()
                        }
                    ),
                    InputOutcome::Uart {
                        accepted: FRAME.len(),
                        dropped: 0
                    }
                );
            }
            match progress {
                Ok(Some(advance)) if advance.bus_request_pending => continue,
                Ok(Some(_)) => break,
                Ok(None) => panic!("нет абсолютной шкалы raw flash"),
                Err(error) => {
                    barrier = Some(error);
                    break;
                }
            }
        }
        if barrier.is_some() {
            break;
        }
        last_sleep = board.esp_sleep_status();
        let now = qns(&*board, board.cycles());
        if let Some(level) = board.pin_output(GPIO_OUT)
            && level != last_gpio
        {
            gpio(&mut *board, GPIO_IN, level);
            gpio_edges.push((level, now));
            last_gpio = level;
        }
        if let Some(config) = board.pwm(PWM) {
            assert!(!pwm_stopped, "ШИМ снова включился после остановки");
            assert_eq!(config.period_s, 1.0 / 4000.0);
            let stage = match config.duty {
                0.25 => 0,
                0.75 => 1,
                duty => panic!("неожиданное заполнение {duty}"),
            };
            let level = board.pin_output(PWM).unwrap();
            if pwm_stage == Some(stage) {
                if level != last_pwm {
                    pwm_edges[stage].push((level, now));
                }
            } else {
                // Переходный период не участвует в измерении длительности.
                assert_eq!(stage, pwm_stage.map_or(0, |previous| previous + 1));
                pwm_stage = Some(stage);
            }
            last_pwm = level;
        } else if pwm_stage.is_some() {
            assert_eq!(board.pin_output(PWM), Some(false));
            pwm_stopped = true;
        }
    }

    let text = std::str::from_utf8(&uart).unwrap();
    for line in [
        "FIXTURE gpio out=0 in=0",
        "FIXTURE gpio out=1 in=1",
        "FIXTURE uart tx=4 rx=4 55 a6 00 ff match",
        "FIXTURE ledc begin hz=4000 duty=256,768",
        "FIXTURE ledc readback hz=4000 duty=768",
        "FIXTURE i2c probe 0x50 ESP_OK",
        "FIXTURE i2c probe 0x51 ESP_ERR_NOT_FOUND",
        "FIXTURE i2c data 5a a5",
        "FIXTURE sleep timer us=2000000 failures=0x0",
    ] {
        assert!(
            text.lines().any(|actual| actual == line),
            "нет {line:?}\n{text}"
        );
    }
    assert_eq!(
        text.lines()
            .filter(|&line| line == "FIXTURE gpio out=0 in=0")
            .count(),
        2
    );
    let [(true, up), (false, down)] = gpio_edges.as_slice() else {
        panic!("фронты GPIO3: {gpio_edges:?}")
    };
    // vTaskDelay(2) при 100 Гц, шаг опроса и не более одной фазы тика.
    assert!((down - up).abs_diff(20 * MS) <= 10 * MS + POLL);
    assert!(PERIPHERALS_QEMU.contains("GPIO write 0x60004008 0x8 4\nGPIO write 0x6000400c 0x8 4"));
    eprintln!(
        "GPIO3 → GPIO10: 0 → 1 → 0, высокий уровень {:.3} мс",
        (down - up) as f64 / MS as f64
    );

    assert!(pwm_stopped);
    assert!(PERIPHERALS_QEMU.contains("LEDC duty 0x60019008 0x1000 0x3000 [esp32c3.iomem]"));
    check_pwm(&pwm_edges[0], 256);
    check_pwm(&pwm_edges[1], 768);
    check_uart(&frames);
    check_i2c(&requests, &done);

    // ELF 2.21, rtc_deep_sleep_start: запись CRC_START в 0x600c0048.
    // Успех этого теста требует явного отказа до SLEEP_EN.
    let expected = BoardFault::Unsupported(
        "сон ESP32-C3: CRC RTC FAST: SYSTEM_RTC_FASTMEM_CONFIG_REG 0x600c0048; TRM рег. 16.11–16.12 не задаёт алгоритм и время расчёта"
            .into(),
    );
    assert_eq!(barrier, Some(expected.clone()), "{text}");
    assert_eq!(board.esp_boot_position().unwrap().pc, 0x4201_764e);
    assert_eq!(board.run_cycles(1), Err(expected));
    let sleep = last_sleep.unwrap();
    assert_eq!((sleep.entered_at, sleep.last_wake), (None, None));
    assert!(
        !text
            .lines()
            .any(|line| line.starts_with("FIXTURE wake ") || line == "FIXTURE done")
    );
    assert!(board.capabilities().blocks_hardware_proof);
    eprintln!("NotAnalyzed: аппаратный CRC RTC FAST 0x600c0048 и оба штатных пробуждения 2.21.");
}

#[test]
fn старые_elf_и_intel_hex_сохраняют_штатную_загрузку() {
    // Прежние значения стенда из boards.rs и исходников firmware-tests.
    for image in [
        include_bytes!("../testdata/avr.elf").as_slice(),
        include_bytes!("../testdata/avr.hex").as_slice(),
    ] {
        let mut board = new_board(BoardId::Atmega328p);
        board.set_pin_input(7, true);
        board.load(image).unwrap();
        board.set_adc_volts(0, 2.5);
        board.run_for(0.0009).unwrap();
        assert_eq!(board.uart_take(), [2, 0]);
        assert_eq!(board.pwm_duty(11), Some(0.25));
    }
    // Т2, DEPTH_PLAN: контрольный ELF ESP32-C3 для положительной загрузки.
    let mut old = new_board(BoardId::Esp32C3);
    old.load(include_bytes!("../testdata/esp.elf")).unwrap();
    assert_eq!(
        old.run_cycles(76),
        Err(BoardFault::Breakpoint { pc: 0x4038_01d2 })
    );
    assert_eq!(old.cycles(), 75);
    assert!(old.esp_boot_position().is_none());
}
