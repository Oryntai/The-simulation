use sim_boards::esp_adc::{EspAdcAttenuation, EspAdcCalibration, EspAdcError};
use sim_boards::{
    BoardFault, BoardId, BoardInput, BoardOutput, EspRadioAbi as Abi, EspRadioPort, InputEvent,
    InputOutcome, RadioStatus, new_board,
};

#[test]
fn adc_без_заводской_калибровки_остаётся_открытым_и_проверяет_границы() {
    // ESP-IDF efuse_reg.h: регистры чтения от 0x2c до 0x178 включительно, DATA4 на 0x6c.
    let mut readback = [0; (0x178 - 0x2c) / size_of::<u32>() + 1];
    assert_eq!(
        EspAdcCalibration::from_efuse_readback(None),
        Err(EspAdcError::NotAnalyzed {
            block_version: None
        })
    );
    assert_eq!(
        EspAdcCalibration::from_efuse_readback(Some(&readback)),
        Err(EspAdcError::NotAnalyzed {
            block_version: Some(0)
        })
    );
    // Там же: BLK_VERSION_MAJOR=1. Нулевые поля смещения версии 1 допустимы.
    readback[(0x6c - 0x2c) / size_of::<u32>()] = 1;
    let calibration = EspAdcCalibration::from_efuse_readback(Some(&readback)).unwrap();
    assert_eq!(calibration.point(EspAdcAttenuation::Db0).reference_mv, 400); // ESP-IDF esp_efuse_rtc_calib.h.
    // ESP32-C3 Series Datasheet v2.5, §5.5, табл. 5-6. Верхние границы в мВ.
    for (attenuation, upper_mv) in [
        (EspAdcAttenuation::Db0, 750.0_f64),
        (EspAdcAttenuation::Db2_5, 1050.0),
        (EspAdcAttenuation::Db6, 1300.0),
        (EspAdcAttenuation::Db12, 2500.0),
    ] {
        assert_eq!(attenuation.validate_input_mv(upper_mv), Ok(()));
        for value in [0.0_f64.next_down(), upper_mv.next_up()] {
            assert_eq!(
                attenuation.validate_input_mv(value),
                Err(EspAdcError::OutsideEffectiveRange(attenuation))
            );
        }
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert_eq!(
                attenuation.validate_input_mv(value),
                Err(EspAdcError::NonFiniteInput)
            );
        }
    }
}

fn radio_program(operation: u32) -> Vec<u8> {
    // ELF32 LE, System V ABI §§4–5: заголовки 52 + 32 байта, один PT_LOAD.
    // ESP32-C3 TRM §3.3.2: SRAM1 через IBUS и DBUS, раздельные адреса кода и команды.
    const IMAGE_OFFSET: usize = 52 + 32;
    const PC: u32 = 0x4038_0000;
    const MAILBOX: u32 = 0x3fc8_1000;
    let mut code = vec![MAILBOX | (5 << 7) | 0x37]; // lui t0, адрес команды.
    let write = |code: &mut Vec<u32>, offset: usize, value: u32, funct3: u32| {
        // RISC-V Unprivileged ISA, гл. 2: addi t1, zero, value, затем sb/sw t1, offset(t0).
        code.push(value << 20 | 6 << 7 | 0x13);
        let offset = offset as u32;
        code.push(
            (offset >> 5) << 25 | 6 << 20 | 5 << 15 | funct3 << 12 | (offset & 31) << 7 | 0x23,
        );
    };
    write(&mut code, Abi::PORT, EspRadioPort::EspNow as u32, 2);
    write(&mut code, Abi::FUNCTION, Abi::OPEN, 2);
    code.extend([Abi::INSTRUCTION, MAILBOX]);
    // Тело команды: передать один байт 0x42 либо вызвать неизвестную функцию.
    write(&mut code, Abi::PAYLOAD_OFFSET, 0x42, 0);
    write(&mut code, Abi::LENGTH, 1, 2);
    write(&mut code, Abi::FUNCTION, operation, 2);
    code.extend([Abi::INSTRUCTION, MAILBOX, 0x0000_006f]); // jal zero, 0.

    let mut elf = vec![0; IMAGE_OFFSET];
    elf[..7].copy_from_slice(b"\x7fELF\x01\x01\x01");
    for (offset, value) in [(16, 2_u16), (18, 243), (40, 52), (42, 32), (44, 1)] {
        elf[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
    }
    let size = (code.len() * size_of::<u32>()) as u32;
    for (offset, value) in [
        (20, 1),
        (24, PC),
        (28, 52),
        (52, 1),
        (56, IMAGE_OFFSET as u32),
        (60, PC),
        (64, PC),
        (68, size),
        (72, size),
        (76, 5),
        (80, 4),
    ] {
        elf[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
    }
    elf.extend(code.into_iter().flat_map(u32::to_le_bytes));
    elf
}

#[test]
fn неизвестная_функция_и_неподтверждённая_передача_различимы_в_журнале() {
    // 100 тактов достаточно для прямолинейного участка стенда RV32I.
    const BUDGET: u64 = 100;
    let port = EspRadioPort::EspNow as u32;
    let mut board = new_board(BoardId::Esp32C3);
    board.load(&radio_program(Abi::ACK + 1)).unwrap(); // Следующий код вне ABI.
    assert!(
        matches!(board.run_cycles(BUDGET), Err(BoardFault::Unsupported(reason)) if reason.contains("неизвестная функция"))
    );
    let events = board.take_events();
    assert!(
        events
            .iter()
            .any(|event| matches!(event.output, BoardOutput::Fault(_)))
    );
    assert!(
        !events
            .iter()
            .any(|event| matches!(event.output, BoardOutput::Radio { .. }))
    );

    board.load(&radio_program(Abi::SEND)).unwrap();
    assert!(board.run_cycles(BUDGET).unwrap().bus_request_pending);
    let events = board.take_events();
    let sent = events
        .iter()
        .find_map(|event| match &event.output {
            BoardOutput::Radio {
                port: actual,
                packet,
            } if *actual == port && packet.bytes == [0x42] => Some(event.cycle),
            _ => None,
        })
        .expect("передача одного байта");
    assert!(events.iter().any(|event| event.cycle == sent
        && matches!(event.output,
        BoardOutput::RadioStatus { port: actual, sent_cycle, status: RadioStatus::Pending }
            if actual == port && sent_cycle == sent)));
    let cycle = board.cycles();
    assert_eq!(
        board
            .apply(InputEvent {
                cycle,
                input: BoardInput::RadioResult {
                    port,
                    sent_cycle: sent - 1,
                    status: RadioStatus::ApplicationConfirmed,
                }
            })
            .unwrap(),
        InputOutcome::Unchanged
    );
    assert!(board.take_events().is_empty());
    board.reset();
    assert!(
        board
            .take_events()
            .iter()
            .any(|event| matches!(event.output,
        BoardOutput::RadioStatus { port: actual, sent_cycle, status: RadioStatus::NotAnalyzed(_) }
            if actual == port && sent_cycle == sent))
    );
}
