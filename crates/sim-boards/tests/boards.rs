use sim_boards::{
    BoardFault, BoardId, Connection, Outcome, SupplyRail, WiringRule, check_wiring, new_board,
    reference_checks,
};

#[test]
fn образы_четырёх_плат_совпали_с_эталоном() {
    let checks = reference_checks();
    assert_eq!(checks.len(), 20);
    let failures: Vec<_> = checks
        .iter()
        .filter(|check| !check.passed())
        .map(|c| {
            format!(
                "{}: получено {}, ожидалось {} {}. {}",
                c.title, c.simulated, c.reference, c.unit, c.note
            )
        })
        .collect();
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

#[test]
fn avr_elf_проходит_по_тому_же_входу_что_hex() {
    let mut avr = new_board(BoardId::Atmega328p);
    avr.set_pin_input(7, true);
    avr.load(include_bytes!("../testdata/avr.elf")).unwrap();
    avr.set_adc_volts(0, 2.5);
    avr.run_for(0.0009).unwrap();
    assert_eq!(avr.uart_take(), [2, 0]);
    assert_eq!(avr.pwm_duty(11), Some(0.25));
}

#[test]
fn опасные_соединения_имеют_свои_причины() {
    let cases = [
        (
            Connection::DigitalInput {
                pin: 0,
                low_v: 0.0,
                high_v: 5.0,
            },
            WiringRule::Overvoltage,
        ),
        (
            Connection::Led {
                pin: 2,
                resistance_ohm: None,
                min_forward_v: 1.7,
                max_drive_v: 3.3,
                sinking: false,
            },
            WiringRule::LedResistor,
        ),
        (
            Connection::Motor {
                pin: 5,
                direct: true,
            },
            WiringRule::DirectMotor,
        ),
        (
            Connection::Supply {
                rail: SupplyRail::Vsys,
                volts: 12.0,
            },
            WiringRule::Supply,
        ),
        (
            Connection::AnalogInput {
                pin: 26,
                volts: 2.0,
                reference_v: Some(1.8),
            },
            WiringRule::AdcRange,
        ),
        (
            Connection::Ground {
                shared: Some(false),
            },
            WiringRule::Ground,
        ),
        (
            Connection::Load {
                pin: 0,
                milliamps: 800.0,
                sinking: false,
            },
            WiringRule::PinCurrent,
        ),
        (
            Connection::Load {
                pin: 0,
                milliamps: f64::INFINITY,
                sinking: false,
            },
            WiringRule::Numeric,
        ),
        (
            Connection::AnalogInput {
                pin: 26,
                volts: 1.0,
                reference_v: Some(f64::NAN),
            },
            WiringRule::Numeric,
        ),
    ];
    for (connection, rule) in cases {
        let findings = check_wiring(BoardId::Rp2040, &[connection]);
        assert!(
            findings
                .iter()
                .any(|f| f.rule == rule && f.outcome == Outcome::Fail),
            "{connection:?}: {findings:?}"
        );
        assert!(
            !findings
                .iter()
                .any(|f| f.connection == Some(0) && f.outcome == Outcome::Pass),
            "{findings:?}"
        );
    }
    let contention = check_wiring(
        BoardId::Rp2040,
        &[
            Connection::Output { pin: 0, net: 1 },
            Connection::Output { pin: 1, net: 1 },
        ],
    );
    assert!(
        contention
            .iter()
            .any(|f| f.rule == WiringRule::Contention && f.outcome == Outcome::Fail)
    );
    let strap = check_wiring(
        BoardId::Esp32C3,
        &[Connection::Strap {
            pin: 9,
            compatible_at_reset: Some(false),
        }],
    );
    assert!(
        strap
            .iter()
            .any(|f| f.rule == WiringRule::Strap && f.outcome == Outcome::Fail)
    );
    assert!(
        check_wiring(BoardId::Rp2040, &[])
            .iter()
            .all(|f| f.outcome == Outcome::NotAnalyzed)
    );
}

fn elf(mut data: Vec<u8>, machine: u16, base: u32, entry: u32) -> Vec<u8> {
    // System V ABI, один PT_LOAD; конструктор для коротких отрицательных проб.
    let mut b = vec![0; 84];
    b[..7].copy_from_slice(b"\x7fELF\x01\x01\x01");
    b[16..18].copy_from_slice(&2u16.to_le_bytes());
    b[18..20].copy_from_slice(&machine.to_le_bytes());
    b[20..24].copy_from_slice(&1u32.to_le_bytes());
    b[24..28].copy_from_slice(&entry.to_le_bytes());
    b[28..32].copy_from_slice(&52u32.to_le_bytes());
    b[40..42].copy_from_slice(&52u16.to_le_bytes());
    b[42..44].copy_from_slice(&32u16.to_le_bytes());
    b[44..46].copy_from_slice(&1u16.to_le_bytes());
    for (i, v) in [
        1,
        84,
        base,
        base,
        data.len() as u32,
        data.len() as u32,
        5,
        4,
    ]
    .into_iter()
    .enumerate()
    {
        b[52 + i * 4..56 + i * 4].copy_from_slice(&v.to_le_bytes());
    }
    b.append(&mut data);
    b
}

fn thumb_elf(id: BoardId, words: &[u16]) -> Vec<u8> {
    let (base, stack) = if id == BoardId::Rp2040 {
        (0x1000_0000u32, 0x2004_2000u32)
    } else {
        (0x0800_0000u32, 0x2000_5000u32)
    };
    let entry = base + 9;
    let mut data = stack.to_le_bytes().to_vec();
    data.extend(entry.to_le_bytes());
    data.extend(words.iter().flat_map(|h| h.to_le_bytes()));
    elf(data, 40, base, entry)
}

#[test]
fn неизвестная_инструкция_останавливает_каждое_ядро() {
    // DDI0419 UDF #0; DS40002198C незанятый opcode 0001; RV32IMC нулевая кодировка запрещена.
    for (id, fw) in [
        (BoardId::Rp2040, thumb_elf(BoardId::Rp2040, &[0xde00])),
        (
            BoardId::Stm32f103c8,
            thumb_elf(BoardId::Stm32f103c8, &[0xde00]),
        ),
        (BoardId::Atmega328p, elf(vec![1, 0], 83, 0, 0)),
        (
            BoardId::Esp32C3,
            elf(vec![0, 0], 243, 0x4038_0000, 0x4038_0000),
        ),
    ] {
        let mut board = new_board(id);
        board.load(&fw).unwrap();
        let error = board.run_for(0.00001).unwrap_err();
        assert!(
            matches!(error, BoardFault::UnknownInstruction { .. }),
            "{id:?}: {error}"
        );
        assert_eq!(board.run_for(0.0), Err(error));
    }
}

#[test]
fn чтение_несуществующего_регистра_даёт_board_fault() {
    let mut board = new_board(BoardId::Rp2040);
    // LDR r0,[PC,#0] / LDR r1,[r0,#0] / адрес за картой SIO. DDI0419.
    board
        .load(&thumb_elf(
            BoardId::Rp2040,
            &[0x4800, 0x6801, 0x00fc, 0xd000],
        ))
        .unwrap();
    assert!(matches!(
        board.run_for(0.001),
        Err(BoardFault::UnsupportedRegister {
            addr: 0xd000_00fc,
            write: false,
            ..
        })
    ));
    assert_eq!(board.pin_output(0), None);
}

#[test]
fn неконечное_время_и_напряжение_отклонены() {
    let mut board = new_board(BoardId::Rp2040);
    board.load(include_bytes!("../testdata/pico.elf")).unwrap();
    assert!(matches!(
        board.run_for(f64::INFINITY),
        Err(BoardFault::Misconfigured(_))
    ));
    board.reset();
    board.set_adc_volts(0, f64::NAN);
    assert!(matches!(
        board.run_for(0.000001),
        Err(BoardFault::Misconfigured(_))
    ));
}

#[test]
fn прошивки_проверяют_цифровой_вход() {
    for (id, image, pin) in [
        (
            BoardId::Rp2040,
            &include_bytes!("../testdata/pico.elf")[..],
            3,
        ),
        (
            BoardId::Stm32f103c8,
            &include_bytes!("../testdata/stm.elf")[..],
            7,
        ),
        (
            BoardId::Esp32C3,
            &include_bytes!("../testdata/esp.elf")[..],
            10,
        ),
        (
            BoardId::Atmega328p,
            &include_bytes!("../testdata/avr.hex")[..],
            7,
        ),
    ] {
        let mut board = new_board(id);
        board.load(image).unwrap();
        board.set_pin_input(pin, false);
        let error = board.run_for(0.001).unwrap_err();
        assert!(
            matches!(error, BoardFault::Breakpoint { .. }),
            "{id:?}: {error}"
        );
    }
}

#[test]
fn несуществующие_регистры_не_отвечают_нулём_или_успехом() {
    // PM0056: LDR r0,литерал / STR r1,[r0]; DS40002198C: STS 0x00ff,r16.
    // RV32I: LUI t0,0x60041 / SW zero,0(t0).
    let cases = [
        (
            BoardId::Stm32f103c8,
            thumb_elf(BoardId::Stm32f103c8, &[0x4800, 0x6001, 0x103c, 0x4002]),
            0x4002_103c,
        ),
        (
            BoardId::Atmega328p,
            elf(vec![0x00, 0x93, 0xff, 0x00], 83, 0, 0),
            0x0000_00ff,
        ),
        (
            BoardId::Esp32C3,
            elf(
                [0x6004_12b7u32, 0x0002_a023]
                    .into_iter()
                    .flat_map(u32::to_le_bytes)
                    .collect(),
                243,
                0x4038_0000,
                0x4038_0000,
            ),
            0x6004_1000,
        ),
    ];
    for (id, image, expected) in cases {
        let mut board = new_board(id);
        board.load(&image).unwrap();
        let error = board.run_for(0.00001).unwrap_err();
        assert!(
            matches!(error, BoardFault::UnsupportedRegister { addr, write: true, .. }
            if addr == expected),
            "{id:?}: {error}"
        );
    }
}

#[test]
fn ток_группы_проверяется_независимо_от_предела_вывода() {
    use sim_boards::wiring_outcome;
    let avr: Vec<_> = (5..=13)
        .map(|pin| Connection::Load {
            pin,
            milliamps: 12.0,
            sinking: true,
        })
        .collect();
    let results = check_wiring(BoardId::Atmega328p, &avr);
    assert!(
        results
            .iter()
            .any(|f| f.rule == WiringRule::TotalCurrent && f.outcome == Outcome::Fail)
    );
    assert!(
        results
            .iter()
            .all(|f| f.rule != WiringRule::PinCurrent || f.outcome == Outcome::Pass)
    );
    assert_eq!(wiring_outcome(&results), Outcome::Fail);
    let pico: Vec<_> = (0..=22)
        .chain(26..=28)
        .map(|pin| Connection::Load {
            pin,
            milliamps: 2.0,
            sinking: false,
        })
        .collect();
    assert!(
        check_wiring(BoardId::Rp2040, &pico)
            .iter()
            .any(|f| f.rule == WiringRule::TotalCurrent && f.outcome == Outcome::Fail)
    );
}

#[test]
fn повреждённый_образ_не_оставляет_старую_прошивку_для_запуска() {
    let mut board = new_board(BoardId::Rp2040);
    board.load(include_bytes!("../testdata/pico.elf")).unwrap();
    let err = board.load(b"ELF").unwrap_err();
    assert!(matches!(err, BoardFault::BadImage(_)));
    assert_eq!(board.run_for(0.00001), Err(err));
    assert_eq!(board.pin_output(25), None);
}
