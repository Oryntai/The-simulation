use sim_boards::{
    BitOrder, BoardCatalog, BoardFault, BoardId, BoardInput, BoardOutput, BoardSim, I2cRequest,
    InputEvent, InputOutcome, OutputEvent, Packet, ResetCause, SpiMode, SpiRequest, TransferStatus,
    capabilities, check_wiring, new_board, wiring_outcome,
};

const ESP_BUS: &[u8] = include_bytes!("../testdata/esp_bus.elf");
const ESP_UART: &[u8] = include_bytes!("../testdata/esp_uart.elf");
const VMCU: &[u8] = include_bytes!("../testdata/vmcu_ports.elf");

fn input(board: &mut dyn BoardSim, input: BoardInput) -> Result<InputOutcome, BoardFault> {
    board.apply(InputEvent {
        cycle: board.cycles(),
        input,
    })
}

fn run_to(board: &mut dyn BoardSim, cycle: u64) -> Vec<OutputEvent> {
    let result = board.run_cycles(cycle - board.cycles()).unwrap();
    assert_eq!(result.cycle, cycle);
    assert!(!result.bus_request_pending);
    board.take_events()
}

fn start_bus(bad_spi: bool) -> (Box<dyn BoardSim>, u64, I2cRequest) {
    let mut board = new_board(BoardId::Esp32C3);
    input(
        &mut *board,
        BoardInput::Gpio {
            pin: 0,
            high: bad_spi,
        },
    )
    .unwrap();
    board.load(ESP_BUS).unwrap();
    let result = board.run_cycles(100_000).unwrap();
    assert!(result.bus_request_pending);
    assert!(result.cycle < 100_000);
    assert_eq!(board.run_cycles(100).unwrap(), result);
    let [event] = board.take_events().try_into().unwrap();
    let BoardOutput::I2c(req) = event.output else {
        panic!("{event:?}")
    };
    (board, event.cycle, req)
}

fn i2c_response(req: &I2cRequest, bytes: &[u8]) -> BoardInput {
    BoardInput::I2cReply {
        bus: 0,
        id: req.id,
        status: TransferStatus::Complete,
        bytes: bytes.into(),
    }
}

fn spi_response(req: &SpiRequest, status: TransferStatus, bytes: &[u8]) -> BoardInput {
    BoardInput::SpiReply {
        bus: 2,
        id: req.id,
        status,
        bytes: bytes.into(),
    }
}

fn transmitted(events: &[OutputEvent]) -> Vec<(u64, u8)> {
    events
        .iter()
        .filter_map(|e| match e.output {
            BoardOutput::Uart { port: 0, byte } => Some((e.cycle, byte)),
            _ => None,
        })
        .collect()
}

#[test]
fn просадка_снимает_выходы_и_перезапускает_тот_же_образ() {
    use sim_boards::GpioMode;

    let mut board = new_board(BoardId::Virtual);
    board.load(VMCU).unwrap();
    // Образ начинает с UART, затем GPIO3 копирует вход GPIO0 и запускается PWM0.
    let mut config = vec![b'S', 0, 0, 0];
    config.extend(1000_u32.to_le_bytes());
    board.write_config(&config).unwrap();
    input(&mut *board, BoardInput::Gpio { pin: 0, high: true }).unwrap();
    input(
        &mut *board,
        BoardInput::Adc {
            channel: 0,
            volts: 1.65,
        },
    )
    .unwrap();
    let first = run_to(&mut *board, 1600);
    assert_eq!(board.gpio_mode(3), Some(GpioMode::Output(true)));
    assert!(board.pwm(0).is_some());
    assert_eq!(
        transmitted(&first).iter().map(|x| x.1).collect::<Vec<_>>(),
        b"S"
    );

    input(&mut *board, BoardInput::Brownout { asserted: true }).unwrap();
    assert_eq!(board.reset_cause(), Some(ResetCause::Brownout));
    assert_eq!(board.gpio_mode(3), None);
    assert_eq!(board.pwm(0), None);
    assert!(matches!(
        input(&mut *board, BoardInput::Power { on: true }),
        Err(BoardFault::Unpowered)
    ));
    let mut held = board.clone();
    let during = run_to(&mut *board, 3200);
    assert_eq!(during, run_to(&mut *held, 3200));
    assert!(transmitted(&during).is_empty());
    assert!(during.iter().any(
        |event| event.cycle == 1600 && event.output == BoardOutput::Reset(ResetCause::Brownout)
    ));

    input(&mut *board, BoardInput::Brownout { asserted: false }).unwrap();
    assert_eq!(board.gpio_mode(3), Some(GpioMode::Input));
    assert_eq!(board.pwm(0), None);
    let restarted = transmitted(&run_to(&mut *board, 4800));
    assert_eq!(
        restarted,
        transmitted(&first)
            .into_iter()
            .map(|(cycle, byte)| (cycle + 3200, byte))
            .collect::<Vec<_>>()
    );
    assert_eq!(board.gpio_mode(3), Some(GpioMode::Output(true)));
    assert!(board.pwm(0).is_some());
    assert!(run_to(&mut *held, 4800).is_empty());
    assert!(!held.powered());
}

#[test]
fn просадка_аппаратной_платы_отменяет_запрос_и_загружает_образ_снова() {
    let (mut board, request_at, request) = start_bus(false);
    let at = board.cycles();
    input(&mut *board, BoardInput::Brownout { asserted: true }).unwrap();
    let events = board.take_events();
    assert!(
        events
            .iter()
            .any(|event| event.cycle == at
                && event.output == BoardOutput::Reset(ResetCause::Brownout))
    );
    assert!(events.iter().any(|event| {
        matches!(event.output, BoardOutput::I2cDone { id, status: TransferStatus::Cancelled, .. } if id == request.id)
    }));
    assert!(matches!(
        input(&mut *board, i2c_response(&request, b"x")),
        Err(BoardFault::Unpowered)
    ));
    // Промежуток без питания длиной как до первого запроса образа.
    assert!(run_to(&mut *board, at * 2).is_empty());
    input(&mut *board, BoardInput::Brownout { asserted: false }).unwrap();
    let result = board.run_cycles(at * 2).unwrap();
    assert!(result.bus_request_pending);
    assert_eq!(result.cycle, at * 3);
    let events = board.take_events();
    assert!(
        events.iter().any(|event| event.cycle == at * 2
            && event.output == BoardOutput::Reset(ResetCause::PowerOn))
    );
    let (second_at, next) = events
        .iter()
        .find_map(|event| match &event.output {
            BoardOutput::I2c(value) => Some((event.cycle, value)),
            _ => None,
        })
        .unwrap();
    assert_ne!(next.id, request.id);
    assert_eq!(second_at, at * 2 + request_at);
    assert_eq!(
        (&next.write, next.read_len, next.address),
        (&request.write, request.read_len, request.address)
    );
}

fn spi_request(board: &mut dyn BoardSim) -> (Vec<OutputEvent>, u64, SpiRequest) {
    let result = board.run_cycles(100_000).unwrap();
    assert!(result.bus_request_pending);
    let events = board.take_events();
    let event = events.last().unwrap();
    let BoardOutput::Spi(request) = &event.output else {
        panic!("{event:?}")
    };
    let (cycle, request) = (event.cycle, request.clone());
    (events, cycle, request)
}

#[test]
fn каталог_расширяется_данными_и_отклоняет_ошибки() {
    let builtin = BoardCatalog::builtin().unwrap();
    let keys = ["esp32-c3", "rp2040", "stm32f103c8", "atmega328p", "vmcu-c3"];
    for id in keys {
        assert_eq!(
            builtin.create(id).unwrap().board(),
            builtin.get(id).unwrap().profile
        );
    }
    let alias = r#"
[[board]]
id = "стенд"
profile = "vmcu-c3"
name = "Нейтральный стенд"
source = "Карта памяти"
"#;
    assert!(BoardCatalog::parse(alias).is_err());
    let alias = alias.replace("стенд\"", "bench-2\"");
    let catalog = BoardCatalog::parse(&alias).unwrap();
    assert_eq!(catalog.create("bench-2").unwrap().board(), BoardId::Virtual);
    assert!(catalog.create("vmcu-c3").is_err());
    for bad in [
        format!("{alias}{alias}"),
        format!("{alias}extra = \"x\""),
        format!("{alias}source = \"duplicate\""),
        alias.replace("vmcu-c3", "未知"),
        alias.replace("name = \"Нейтральный bench-2\"", ""),
        alias.replace("source = \"Карта памяти\"", "source = false"),
        "[[board]]".to_string(),
    ] {
        assert!(BoardCatalog::parse(&bad).is_err(), "{bad}");
    }
    assert_eq!(
        wiring_outcome(&check_wiring(BoardId::Virtual, &[])),
        sim_boards::Outcome::NotAnalyzed
    );
}

#[test]
fn i2c_repeated_start_и_spi_дуплекс_дают_байты_в_исполняемом_elf() {
    let (mut board, at, req) = start_bus(false);
    assert_eq!(
        (
            req.bus,
            req.address,
            req.write.as_slice(),
            req.read_len,
            req.repeated_start
        ),
        (0, 0x30, &[0x24][..], 2, true)
    );
    // Стенд: 100 кГц, 5 байтовых кадров по 9 бит, START 10, restart 10+14, STOP 18+22.
    assert_eq!(req.cycles_per_bit, 80_000_000 / 100_000);
    assert_eq!(req.address_ack_cycle - at, 10 + 9 * 800);
    assert_eq!(req.complete_cycle - at, 10 + 5 * 9 * 800 + 24 + 40);
    assert!(input(&mut *board, i2c_response(&req, &[0x90])).is_err());
    let mut wrong_id = req.clone();
    wrong_id.id += 1;
    assert!(input(&mut *board, i2c_response(&wrong_id, &[0x90, 0x81])).is_err());
    input(&mut *board, i2c_response(&req, &[0x90, 0x81])).unwrap();
    assert!(input(&mut *board, i2c_response(&req, &[0, 0])).is_err());
    assert!(run_to(&mut *board, req.complete_cycle - 1).is_empty());

    let (events, at, spi) = spi_request(&mut *board);
    assert_eq!(
        events[0],
        OutputEvent {
            cycle: req.complete_cycle,
            output: BoardOutput::I2cDone {
                bus: 0,
                id: req.id,
                address: 0x30,
                status: TransferStatus::Complete,
                bytes: vec![0x90, 0x81]
            },
        }
    );
    assert_eq!(spi.bus, 2);
    assert_eq!(spi.mode, SpiMode::Mode1);
    assert_eq!(spi.write_order, BitOrder::LsbFirst);
    assert_eq!(spi.read_order, BitOrder::MsbFirst);
    assert_eq!(spi.chip_select, 2);
    assert_eq!(spi.write, [0x53, 0xac]);
    // Стенд: 1 МГц, 31 полупериод между фронтами, CS setup 2.5 и hold 3.5 периода.
    assert_eq!(spi.cycles_per_bit, 80_000_000 / 1_000_000);
    assert_eq!(spi.complete_cycle - at, (31 + 5 + 7) * 40);
    input(
        &mut *board,
        spi_response(&spi, TransferStatus::Complete, &[0x26, 0xe1]),
    )
    .unwrap();
    assert!(run_to(&mut *board, spi.complete_cycle - 1).is_empty());
    let events = run_to(&mut *board, spi.complete_cycle + 120_000);
    assert_eq!(
        events[0],
        OutputEvent {
            cycle: spi.complete_cycle,
            output: BoardOutput::SpiDone {
                bus: 2,
                id: spi.id,
                chip_select: 2,
                status: TransferStatus::Complete,
                bytes: vec![0x26, 0xe1]
            },
        }
    );
    let tx = transmitted(&events);
    assert!(tx[0].0 > req.complete_cycle);
    assert_eq!(
        tx.iter().map(|p| p.1).collect::<Vec<_>>(),
        [0x90, 0x81, 0x26, 0xe1]
    );
    // UART в стенде использует XTAL, два делителя reset 2 и 0x2b6, 10 бит на байт.
    for frames in tx.windows(2) {
        assert_eq!(frames[1].0 - frames[0].0, 2 * 2 * 0x2b6 * 10);
    }
}

#[test]
fn i2c_неизвестный_адрес_получает_nack_и_не_возвращает_данные() {
    for explicit in [false, true] {
        let (mut board, at, req) = start_bus(false);
        if explicit {
            input(
                &mut *board,
                BoardInput::I2cReply {
                    bus: 0,
                    id: req.id,
                    status: TransferStatus::Nack,
                    bytes: Vec::new(),
                },
            )
            .unwrap();
        }
        let nack_cycle = at + 10 + 9 * 800 + 40;
        assert!(run_to(&mut *board, req.address_ack_cycle).is_empty());
        assert!(input(&mut *board, i2c_response(&req, &[1, 2])).is_err());
        assert!(run_to(&mut *board, nack_cycle - 1).is_empty());
        let events = run_to(&mut *board, at + 90_000);
        assert_eq!(
            events[0],
            OutputEvent {
                cycle: nack_cycle,
                output: BoardOutput::I2cDone {
                    bus: 0,
                    id: req.id,
                    address: req.address,
                    status: TransferStatus::Nack,
                    bytes: Vec::new()
                },
            }
        );
        assert_eq!(transmitted(&events)[0].1, b'N');
        assert_eq!(events.len(), 2);
    }
}

#[test]
fn spi_несовместимый_режим_и_отсутствующий_ответ_дают_ошибку() {
    let (mut bad, _, req) = start_bus(true);
    input(&mut *bad, i2c_response(&req, &[1, 2])).unwrap();
    assert!(matches!(
        bad.run_cycles(100_000),
        Err(BoardFault::Unsupported(_))
    ));
    assert!(
        bad.take_events()
            .iter()
            .all(|e| !matches!(e.output, BoardOutput::Spi(_)))
    );

    for explicit in [false, true] {
        let (mut board, _, req) = start_bus(false);
        input(&mut *board, i2c_response(&req, &[1, 2])).unwrap();
        let (_, _, spi) = spi_request(&mut *board);
        assert!(
            input(
                &mut *board,
                spi_response(&spi, TransferStatus::Complete, &[1])
            )
            .is_err()
        );
        if explicit {
            input(
                &mut *board,
                spi_response(&spi, TransferStatus::NoResponse, &[]),
            )
            .unwrap();
        }
        assert!(
            board
                .run_cycles(spi.complete_cycle - board.cycles())
                .is_err()
        );
        assert_eq!(board.cycles(), spi.complete_cycle);
        let events = board.take_events();
        assert_eq!(events.len(), 2);
        assert_eq!(
            events[0],
            OutputEvent {
                cycle: spi.complete_cycle,
                output: BoardOutput::SpiDone {
                    bus: 2,
                    id: spi.id,
                    chip_select: 2,
                    status: TransferStatus::NoResponse,
                    bytes: vec![]
                },
            }
        );
        assert!(matches!(
            events[1].output,
            BoardOutput::Fault(BoardFault::Misconfigured(_))
        ));
    }
}

#[test]
fn копия_сохраняет_транзакцию_и_ответ_и_может_потерять_питание_отдельно() {
    let (mut board, start, i2c) = start_bus(false);
    assert!(run_to(&mut *board, i2c.address_ack_cycle - 1).is_empty());
    let mut copy = board.clone();
    input(&mut *board, i2c_response(&i2c, &[0x42, 0x91])).unwrap();
    input(&mut *copy, i2c_response(&i2c, &[0x42, 0x91])).unwrap();
    let original = spi_request(&mut *board);
    let continuation = spi_request(&mut *copy);
    assert_eq!(original, continuation);

    let (_, start_spi, spi) = original;
    let middle = (start_spi + spi.complete_cycle) / 2;
    assert!(middle > start);
    input(
        &mut *board,
        spi_response(&spi, TransferStatus::Complete, &[0x19, 0xfe]),
    )
    .unwrap();
    assert!(run_to(&mut *board, middle).is_empty());
    let mut independent = board.clone();
    let mut interrupted = board.clone();
    input(&mut *interrupted, BoardInput::Power { on: false }).unwrap();
    assert_eq!(interrupted.pwm(0), None);
    assert_eq!(interrupted.pin_output(0), None);
    assert_eq!(interrupted.run_for(0.0), Err(BoardFault::Unpowered));
    assert_eq!(
        run_to(&mut *interrupted, spi.complete_cycle + 200),
        [
            OutputEvent {
                cycle: middle,
                output: BoardOutput::Power { on: false }
            },
            OutputEvent {
                cycle: middle,
                output: BoardOutput::SpiDone {
                    bus: 2,
                    id: spi.id,
                    chip_select: spi.chip_select,
                    status: TransferStatus::Cancelled,
                    bytes: vec![],
                }
            },
        ]
    );
    let finish = spi.complete_cycle + 120_000;
    assert_eq!(
        run_to(&mut *board, finish),
        run_to(&mut *independent, finish)
    );
    input(&mut *interrupted, BoardInput::Power { on: true }).unwrap();
    assert_eq!(interrupted.reset_cause(), Some(ResetCause::PowerOn));
    assert!(
        input(
            &mut *interrupted,
            spi_response(&spi, TransferStatus::Complete, &[1, 2])
        )
        .is_err()
    );
    let (_, _, next) = {
        assert!(interrupted.run_cycles(100_000).unwrap().bus_request_pending);
        let events = interrupted.take_events();
        let last = events.last().unwrap();
        let BoardOutput::I2c(req) = &last.output else {
            panic!("{last:?}")
        };
        (events.len(), last.cycle, req.clone())
    };
    assert!(next.id > spi.id);
}

#[test]
fn uart_rx_wfi_irq_эхо_и_срок_8n1() {
    let mut board = new_board(BoardId::Esp32C3);
    board.load(ESP_UART).unwrap();
    assert!(run_to(&mut *board, 100).is_empty());
    let frame = 80_000_000 / 1_000_000 * 10;
    let start = board.cycles();
    assert_eq!(
        input(
            &mut *board,
            BoardInput::Uart {
                port: 0,
                bytes: vec![0x00, 0x81, 0xff]
            }
        )
        .unwrap(),
        InputOutcome::Uart {
            accepted: 3,
            dropped: 0
        }
    );
    assert!(run_to(&mut *board, start + frame - 1).is_empty());
    let mut branch = board.clone();
    let events = run_to(&mut *board, start + 5 * frame);
    assert_eq!(events, run_to(&mut *branch, start + 5 * frame));
    let uart = transmitted(&events);
    assert_eq!(
        uart.iter().map(|p| p.1).collect::<Vec<_>>(),
        [0x00, 0x81, 0xff]
    );
    // Два кадра 8N1, между ними вход в IRQ и инструкции обработчика.
    for (index, (cycle, _)) in uart.into_iter().enumerate() {
        let ideal = start + (index as u64 + 2) * frame;
        assert!(
            (ideal + 1..ideal + 40).contains(&cycle),
            "{cycle} против {ideal}"
        );
    }
}

#[test]
fn uart_rx_переполнение_линии_и_fifo_отличаются() {
    let mut board = new_board(BoardId::Esp32C3);
    board.load(ESP_UART).unwrap();
    run_to(&mut *board, 100);
    input(&mut *board, BoardInput::Execution { enabled: false }).unwrap();
    assert_eq!(
        board.take_events(),
        [OutputEvent {
            cycle: 100,
            output: BoardOutput::Execution { enabled: false }
        }]
    );
    let mut bytes: Vec<u8> = (0..128).collect();
    bytes.push(0xaa);
    assert_eq!(
        input(&mut *board, BoardInput::Uart { port: 0, bytes }).unwrap(),
        InputOutcome::Uart {
            accepted: 128,
            dropped: 1
        }
    );
    let frame = 800;
    let full = board.cycles() + 128 * frame;
    assert!(run_to(&mut *board, full).is_empty());
    assert_eq!(
        input(
            &mut *board,
            BoardInput::Uart {
                port: 0,
                bytes: vec![0xbb]
            }
        )
        .unwrap(),
        InputOutcome::Uart {
            accepted: 1,
            dropped: 0
        }
    );
    assert!(run_to(&mut *board, full + frame).is_empty());
    input(&mut *board, BoardInput::Execution { enabled: true }).unwrap();
    let events = run_to(&mut *board, full + 132 * frame);
    let expected = std::iter::once(b'O').chain(0..128).collect::<Vec<_>>();
    assert_eq!(
        transmitted(&events).iter().map(|p| p.1).collect::<Vec<_>>(),
        expected
    );
}

#[test]
fn виртуальный_elf_совпадает_с_прямым_mcu_включая_watchdog_и_пакет() {
    let catalog = BoardCatalog::builtin().unwrap();
    let cpu_hz = 160_000_000; // MEMORY_MAP.md, SYSCTRL.CPU_HZ.
    for watchdog in [false, true] {
        let mut board = catalog.create("vmcu-c3").unwrap();
        let mut direct = sim_mcu::Mcu::from_elf(VMCU).unwrap();
        board.load(VMCU).unwrap();
        board
            .write_config(&[b'A', u8::from(watchdog), 0, 0, 50, 0, 0, 0])
            .unwrap();
        direct
            .write_config(&[b'A', u8::from(watchdog), 0, 0, 50, 0, 0, 0])
            .unwrap();
        board.reset();
        direct.reset();
        input(
            &mut *board,
            BoardInput::Adc {
                channel: 0,
                volts: 1.65,
            },
        )
        .unwrap();
        input(&mut *board, BoardInput::Gpio { pin: 0, high: true }).unwrap();
        direct.set_adc_volts(0, 1.65);
        direct.set_gpio_input(0, true);
        assert_eq!(board.capabilities().cpu_hz, cpu_hz);

        board.run_for(10e-6).unwrap();
        direct.run_for(10e-6).unwrap();
        assert_eq!(board.cycles(), 1600);
        assert_eq!(board.cycles(), direct.cycles());
        assert_eq!(board.pin_output(3), Some(direct.gpio_output(3)));
        assert_eq!(board.pin_output(3), Some(true));
        assert_eq!(board.pwm_duty(0), Some(direct.pwm_duty(0)));
        let pwm = board.pwm(0).unwrap();
        assert_eq!(pwm.duty, 32768.0 / 65535.0); // Полушкала 12 бит, сдвиг стенда на 4.
        assert_eq!(pwm.period_s, 0.02);
        assert_eq!(pwm.high_s, 32768.0 / 65535.0 / 50.0);
        assert_eq!(board.uart_take(), direct.uart_take_bytes());

        let payload = [0x81, 0x00, 0xff];
        if !watchdog {
            assert_eq!(
                input(
                    &mut *board,
                    BoardInput::Uart {
                        port: 0,
                        bytes: payload.to_vec()
                    }
                )
                .unwrap(),
                InputOutcome::Uart {
                    accepted: 3,
                    dropped: 0
                }
            );
            assert_eq!(direct.uart_send(&payload), 3);
            input(
                &mut *board,
                BoardInput::Radio {
                    port: 0,
                    packet: Packet {
                        bytes: vec![0xa7],
                        rssi_dbm: Some(-40),
                    },
                },
            )
            .unwrap();
            assert!(direct.radio_deliver(&[0xa7], -40));
        }
        board.run_cycles(6400).unwrap();
        direct.run_for(40e-6).unwrap();
        assert_eq!(board.cycles(), direct.cycles());
        assert_eq!(board.cycles(), 8000);
        assert_eq!(board.exit_code(), direct.exit_code());
        assert_eq!(board.exit_code(), Some(if watchdog { 2 } else { 0 }));
        assert_eq!(board.watchdog_resets(), direct.watchdog_resets());
        let events = board.take_events();
        let serial: Vec<_> = transmitted(&events).into_iter().map(|e| e.1).collect();
        assert_eq!(serial, direct.uart_take_bytes());
        if watchdog {
            assert_eq!(board.reset_cause(), Some(ResetCause::Watchdog));
            assert_eq!(direct.reset_reason(), sim_mcu::ResetReason::Watchdog);
            let resets: Vec<_> = events
                .iter()
                .filter(|e| e.output == BoardOutput::Reset(ResetCause::Watchdog))
                .collect();
            assert_eq!(resets.len(), 1);
            // Сторож стартует после GPIO, АЦП и PWM; 30 мкс = 4800 тактов.
            assert!((4800..4900).contains(&resets[0].cycle));
        } else {
            assert_eq!(serial, [0x81]);
            let packets: Vec<_> = events
                .iter()
                .filter_map(|e| match &e.output {
                    BoardOutput::Radio { port: 0, packet } => Some(packet.bytes.clone()),
                    _ => None,
                })
                .collect();
            assert_eq!(packets, direct.radio_take_tx());
            assert_eq!(packets, [vec![0xa7, (-40i8) as u8]]);
        }
    }
}

#[test]
fn входы_отклоняют_nan_неточное_время_и_неподдержанный_порт() {
    let mut virtual_board = new_board(BoardId::Virtual);
    virtual_board.load(VMCU).unwrap();
    for volts in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert!(input(&mut *virtual_board, BoardInput::Adc { channel: 0, volts }).is_err());
    }
    assert!(
        virtual_board
            .apply(InputEvent {
                cycle: 1,
                input: BoardInput::Gpio { pin: 0, high: true }
            })
            .is_err()
    );
    assert!(virtual_board.run_cycles(u64::MAX).is_err());
    assert!(
        virtual_board
            .write_config(&vec![0; capabilities(BoardId::Virtual).config_bytes + 1])
            .is_err()
    );
    assert!(matches!(
        virtual_board.run_cycles(1000),
        Err(BoardFault::Execution(_))
    ));
    assert_eq!(virtual_board.pwm(0), None);
    let mut missing_entry = VMCU.to_vec();
    missing_entry[24..28].copy_from_slice(&(sim_mcu::bus::FLASH_BASE + 4096).to_le_bytes());
    assert!(matches!(
        virtual_board.load(&missing_entry),
        Err(BoardFault::BadImage(_))
    ));
    assert!(virtual_board.load(b"bad ELF").is_err());
    assert_eq!(virtual_board.exit_code(), None);
    assert!(virtual_board.run_cycles(1).is_err());

    let mut board = new_board(BoardId::Esp32C3);
    board.load(ESP_UART).unwrap();
    assert!(matches!(
        input(
            &mut *board,
            BoardInput::Radio {
                port: 0,
                packet: Packet {
                    bytes: vec![1],
                    rssi_dbm: Some(-40)
                }
            }
        ),
        Err(BoardFault::Unsupported(_))
    ));
    assert!(board.write_config(&[1]).is_err());
}

#[test]
fn пауза_ядра_не_останавливает_watchdog_и_снимает_выход() {
    use sim_boards::{OutputPort, PortValue};

    let mut board = new_board(BoardId::Virtual);
    board.load(VMCU).unwrap();
    board.write_config(&[b'W', 1, 0, 0, 50, 0, 0, 0]).unwrap();
    input(
        &mut *board,
        BoardInput::Adc {
            channel: 0,
            volts: 1.65,
        },
    )
    .unwrap();
    input(
        &mut *board,
        BoardInput::Gpio {
            pin: 0,
            high: false,
        },
    )
    .unwrap();
    assert_eq!(transmitted(&run_to(&mut *board, 1000)).len(), 1);
    assert_eq!(
        board.sample(OutputPort::Gpio(3)).value,
        Some(PortValue::Logic(false))
    );
    assert!(matches!(
        board.sample(OutputPort::Pwm(0)).value,
        Some(PortValue::Pwm(_))
    ));
    input(&mut *board, BoardInput::Execution { enabled: false }).unwrap();
    assert_eq!(
        input(&mut *board, BoardInput::Execution { enabled: false }).unwrap(),
        InputOutcome::Unchanged
    );
    let mut fork = board.clone();
    let events = run_to(&mut *board, 6000);
    assert_eq!(events, run_to(&mut *fork, 6000));
    let [pause, reset] = &events[..] else {
        panic!("{events:?}")
    };
    assert_eq!(pause.cycle, 1000);
    assert_eq!(pause.output, BoardOutput::Execution { enabled: false });
    assert!((4800..4900).contains(&reset.cycle));
    assert_eq!(reset.output, BoardOutput::Reset(ResetCause::Watchdog));
    assert_eq!(board.watchdog_resets(), 1);
    assert_eq!(board.sample(OutputPort::Pwm(0)).cycle, 6000);
    assert_eq!(board.sample(OutputPort::Pwm(0)).value, None);
    assert_eq!(board.sample(OutputPort::Gpio(3)).value, None);

    input(&mut *board, BoardInput::Execution { enabled: true }).unwrap();
    run_to(&mut *board, 8000);
    assert_eq!(board.exit_code(), Some(2));
    assert_eq!(fork.exit_code(), None);
}

#[test]
fn обрыв_аналогового_входа_убирает_старое_значение() {
    let mut board = new_board(BoardId::Virtual);
    board.load(VMCU).unwrap();
    input(
        &mut *board,
        BoardInput::Adc {
            channel: 0,
            volts: 1.65,
        },
    )
    .unwrap();
    input(&mut *board, BoardInput::DisconnectAdc { channel: 0 }).unwrap();
    assert!(matches!(
        board.run_cycles(1000),
        Err(BoardFault::Execution(_))
    ));
    assert!(
        board
            .take_events()
            .iter()
            .any(|event| matches!(event.output, BoardOutput::Fault(BoardFault::Execution(_))))
    );
    assert_eq!(board.pwm(0), None);
    board.reset();
    input(
        &mut *board,
        BoardInput::Adc {
            channel: 0,
            volts: 1.65,
        },
    )
    .unwrap();
    assert!(board.run_cycles(1000).is_ok());
}

#[test]
fn esp_pwm_метаданные_совпадают_с_отсчётами_фронтов() {
    // Из существующего стенда LEDC: APB 80 МГц, делитель 1, 8 бит, ширина 64.
    let mut board = new_board(BoardId::Esp32C3);
    board.set_pin_input(10, true);
    board.set_adc_volts(0, 1.25);
    board.load(include_bytes!("../testdata/esp.elf")).unwrap();
    board.run_cycles(80_000).unwrap();
    let pwm = board.pwm(4).unwrap();
    assert_eq!(pwm.period_s, 256.0 / 80_000_000.0);
    assert_eq!(pwm.high_s, 64.0 / 80_000_000.0);
    let mut high = 0;
    let mut previous = board.pin_output(4).unwrap();
    let mut rising = Vec::new();
    for index in 1..=3 * 256 {
        board.run_cycles(1).unwrap();
        let level = board.pin_output(4).unwrap();
        if level {
            high += 1;
        }
        if level && !previous {
            rising.push(index);
        }
        previous = level;
    }
    assert_eq!(rising.len(), 3);
    assert!(rising.windows(2).all(|pair| pair[1] - pair[0] == 256));
    assert_eq!(high, 3 * 64);
}
