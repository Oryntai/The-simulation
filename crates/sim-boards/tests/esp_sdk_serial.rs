//! Кадровый UART и I²C по MMIO гостя, включая маршрутизацию и часы raw flash.
#[path = "../src/peripheral/esp_uart.rs"]
mod esp_uart;

use esp_uart::EspUart;
use sim_boards::{
    BoardFault, BoardId, BoardInput, BoardOutput, BoardSim, I2cRequest, InputEvent, InputOutcome,
    OutputEvent, TransferStatus, esp_image, new_board,
};

// uart_reg.h: INT_RAW, INT_ST, INT_ENA, INT_CLR и FIFO/ST.
const UART_RAW: u32 = 0x04;
const UART_STATUS: u32 = 0x1c;
const UART_ENABLE: u32 = 0x0c;
const UART_CLEAR: u32 = 0x10;
const UART_MASKED: u32 = 0x08;
const UART_FULL: u32 = 1;
const UART_OVERFLOW: u32 = 1 << 4;
const UART_TIMEOUT: u32 = 1 << 8;
const UART_TX_DONE: u32 = 1 << 14;

fn uart_set(uart: &mut EspUart, offset: u32, value: u32, at: u64) {
    assert_eq!(uart.access(offset, Some(value), at).unwrap(), Some(0));
}

fn uart_get(uart: &mut EspUart, offset: u32, at: u64) -> u32 {
    uart.access(offset, None, at).unwrap().unwrap()
}

fn uart_idf() -> EspUart {
    // TRM §6.2: APB=80 МГц под PLL, XTAL=40 МГц. RC_FAST не задан.
    let mut uart = EspUart::new(80_000_000, 80_000_000, 40_000_000, None).unwrap();
    // uart_ll_set_baudrate: floor(40 МГц × 16 / 115200) = 5555 = 347×16+3.
    // CLKDIV=0x0030015b также прочитан uart_param_config в ESP-IDF v6.1.1 / QEMU 9.2.2.
    uart_set(&mut uart, 0x14, 0x0030_015b, 0);
    // CLK_CONF: XTAL, NUM=0; IDLE_CONF: TX_IDLE_NUM=0; CONF1: RX_TOUT_EN и RX_FULL=2.
    uart_set(&mut uart, 0x78, 0x0370_0000, 0);
    uart_set(&mut uart, 0x48, 0x100, 0);
    uart_set(&mut uart, 0x24, 0x0020_0002, 0);
    // uart_driver_install: RX FIFO full, timeout, overflow, break, parity, wakeup.
    uart_set(&mut uart, UART_ENABLE, 0x0008_0195, 0);
    uart
}

// Период кадра 8N1 в тактах APB — 2×10×(347+3/16), округление метки вверх.
fn uart_frame(n: u64) -> u64 {
    (n * 20 * 5555).div_ceil(16)
}

#[test]
fn uart_байты_и_порог_по_делителю() {
    let mut uart = uart_idf();
    // Контрольная ESP-IDF v6.1.1 в QEMU: UART_DATA, size=3, timeout_flag=0, 41 42 43.
    assert_eq!(
        uart.receive(0, b"ABC").unwrap(),
        InputOutcome::Uart {
            accepted: 3,
            dropped: 0
        }
    );
    assert_eq!(uart_get(&mut uart, UART_STATUS, uart_frame(1) - 1), 0);
    assert_eq!(uart_get(&mut uart, UART_STATUS, uart_frame(1)), 1);
    assert_eq!(uart_get(&mut uart, UART_STATUS, uart_frame(2)), 2);
    // uart_ll_set_rxfifo_full_thr: строго больше порога. QEMU срабатывает уже на равенстве.
    assert!(!uart.irq());
    assert_eq!(uart_get(&mut uart, UART_MASKED, uart_frame(3)), UART_FULL);
    assert!(uart.irq());
    uart_set(&mut uart, UART_CLEAR, UART_FULL, uart_frame(3));
    assert!(uart.irq()); // Уровневое условие ещё выполняется.
    for &byte in b"ABC" {
        assert_eq!(uart_get(&mut uart, 0, uart_frame(3)), u32::from(byte));
    }
    assert!(!uart.irq());
    assert!(uart.access(0, None, uart_frame(3)).is_err());
    assert_eq!(uart_get(&mut uart, UART_STATUS, uart_frame(4)), 0);
}

#[test]
fn uart_пауза_маска_очистка_и_снимок() {
    let mut uart = uart_idf();
    // Контрольная ESP-IDF v6.1.1 в QEMU: UART_DATA, size=1, timeout_flag=1, 51.
    uart.receive(0, b"Q").unwrap();
    let mut snapshot = uart.clone();
    uart_set(&mut uart, UART_ENABLE, 0, uart_frame(1));
    // MEM_CONF.RX_TOUT_THRHD=10 интервалов: один кадр 8N1 после байта.
    assert_eq!(uart_get(&mut uart, UART_RAW, uart_frame(2) - 1), 0);
    assert_eq!(uart_get(&mut uart, UART_RAW, uart_frame(2)), UART_TIMEOUT);
    assert!(!uart.irq());
    assert_eq!(
        uart_get(&mut snapshot, UART_MASKED, uart_frame(2)),
        UART_TIMEOUT
    );
    assert!(snapshot.irq());
    uart_set(&mut uart, UART_ENABLE, UART_TIMEOUT, uart_frame(2));
    assert!(uart.irq());
    uart_set(&mut uart, UART_CLEAR, UART_TIMEOUT, uart_frame(2));
    assert_eq!(uart_get(&mut uart, UART_RAW, uart_frame(4)), 0);
    assert_eq!(uart_get(&mut uart, 0, uart_frame(4)), u32::from(b'Q'));
    assert!(uart.tick(uart_frame(4) - 1).is_err());
}

#[test]
fn uart_переполнение_fifo_и_ошибки_входа() {
    let mut uart = uart_idf();
    // UART_LL_FIFO_DEF_LEN=128. Ограничение входной очереди отдельно от RX FIFO.
    assert_eq!(
        uart.receive(0, &[b'A'; 129]).unwrap(),
        InputOutcome::Uart {
            accepted: 128,
            dropped: 1
        }
    );
    assert_eq!(uart_get(&mut uart, UART_STATUS, uart_frame(128)), 128);
    assert_eq!(uart_get(&mut uart, UART_RAW, uart_frame(128)), UART_FULL);
    uart.receive(uart_frame(128), b"B").unwrap();
    let at = uart_frame(128) + uart_frame(1);
    assert_eq!(uart_get(&mut uart, UART_RAW, at), UART_FULL | UART_OVERFLOW);
    assert_eq!(uart_get(&mut uart, UART_STATUS, at), 128);
    uart_set(&mut uart, UART_CLEAR, UART_OVERFLOW, at);
    assert_eq!(uart_get(&mut uart, UART_RAW, at), UART_FULL);
    // UART_CONF0.RXFIFO_RST. Пара штатных записей драйвера 1→0.
    uart_set(&mut uart, 0x20, 0x1002_001c, at);
    uart_set(&mut uart, 0x20, 0x1000_001c, at);
    assert_eq!(uart_get(&mut uart, UART_STATUS, at), 0);
    assert!(!uart.irq());

    assert!(EspUart::new(1, 0, 1, None).is_err());
    assert!(uart.receive(u64::MAX, b"A").is_err());
    let mut uart = uart_idf();
    uart_set(&mut uart, 0x78, 0x0360_0000, 0); // SCLK_SEL=RC_FAST, частота неизвестна.
    assert!(matches!(
        uart.receive(0, b"A"),
        Err(BoardFault::Unsupported(_))
    ));
    uart_set(&mut uart, 0x78, 0x0170_0000, 0); // RX_SCLK_EN=0.
    assert!(uart.receive(0, b"A").is_err());
    uart_set(&mut uart, 0x78, 0x0370_0000, 0);
    uart_set(&mut uart, 0x14, 0, 0);
    assert!(uart.receive(0, b"A").is_err());
}

#[test]
fn uart_синхронизация_и_tx_без_накопления_округления() {
    let mut uart = uart_idf();
    // UART_ID.HIGH_SPEED=0, затем UPDATE. До UPDATE действует старый CLKDIV.
    uart_set(&mut uart, 0x80, 0x500, 0);
    uart_set(&mut uart, 0x14, 347, 0);
    uart_set(&mut uart, UART_ENABLE, UART_TX_DONE, 0);
    for &byte in b"ABCDE" {
        uart_set(&mut uart, 0, u32::from(byte), 0);
    }
    assert!(!uart.irq());
    uart.tick(uart_frame(5)).unwrap();
    assert_eq!(
        uart.take_tx(),
        b"ABCDE"
            .iter()
            .enumerate()
            .map(|(i, &byte)| (uart_frame(i as u64 + 1), byte))
            .collect::<Vec<_>>()
    );
    assert!(uart.irq());
    uart_set(&mut uart, 0x80, 0x8000_0500, uart_frame(5));
    let at = uart_frame(5);
    uart_set(&mut uart, 0, u32::from(b'A'), at);
    uart.tick(at + 2 * 347 * 10).unwrap();
    assert_eq!(uart.take_tx(), vec![(at + 2 * 347 * 10, b'A')]);
}

#[test]
fn uart_тайм_аут_не_растёт_во_время_следующего_кадра() {
    let mut uart = uart_idf();
    // UART_MEM_CONF: RX_TOUT_THRHD=1 бит, TX_SIZE=RX_SIZE=1.
    uart_set(&mut uart, 0x60, 1 << 16 | 0x12, 0);
    uart.receive(0, b"AB").unwrap();
    assert_eq!(uart_get(&mut uart, UART_RAW, uart_frame(2) - 1), 0);
    assert_eq!(uart_get(&mut uart, UART_STATUS, uart_frame(2)), 2);
    // Двадцать бит двух кадров с одним дополнительным битом простоя.
    let idle = (21u64 * 2 * 5555).div_ceil(16);
    assert_eq!(uart_get(&mut uart, UART_RAW, idle - 1), 0);
    assert_eq!(uart_get(&mut uart, UART_RAW, idle), UART_TIMEOUT);
}

// soc/esp32c3/register/soc/{reg_base,i2c_reg}.h, i2c_ll.h.
const BASE: u32 = 0x6001_3000;
const CTR: u32 = 0x04;
const SR: u32 = 0x08;
const TO: u32 = 0x0c;
const FIFO: u32 = 0x1c;
const RAW: u32 = 0x20;
const CLR: u32 = 0x24;
const ENA: u32 = 0x28;
const MASKED: u32 = 0x2c;
const COMMAND: u32 = 0x58;
const RESTART: u32 = 6 << 11;
const STOP: u32 = 2 << 11;
const END: u32 = 4 << 11;
const WRITE_ACK: u32 = (1 << 11) | (1 << 8);
const READ_ACK: u32 = 3 << 11;
const READ_NACK: u32 = READ_ACK | (1 << 10);
const MASTER: u32 = 0x13; // MS_MODE, SDA_FORCE_OUT, SCL_FORCE_OUT.
const CONFIG_UPDATE: u32 = 1 << 11;
const MASTER_START: u32 = MASTER | (1 << 5);
const NACK: u32 = 1 << 10;
const TIMEOUT: u32 = 1 << 8;
const COMPLETE: u32 = 1 << 7;
const END_DETECT: u32 = 1 << 3;
const DRIVER_EVENTS: u32 = NACK | TIMEOUT | COMPLETE | (1 << 5) | END_DETECT;
const BUSY: u32 = 1 << 4;
const DONE: u32 = 1 << 31;
const ADDRESS: u8 = 0x30; // Произвольный адрес нейтрального стенда.
const FAST_BIT: u64 = 80_000_000 / 400_000;
// ESP-IDF i2c_ll_calculate_timeout_us_to_reg_val(40 МГц, 125 мкс):
// 5000 тактов XTAL -> 13 бит, аппаратный порог 8192 SCLK, делитель SCLK=1.
const STRETCH_LIMIT: u64 = (1 << 13) * 2;

const ENTRY: u32 = 0x4038_0000; // TRM §3.3: SRAM через шину инструкций.
const EBREAK: u32 = 0x0010_0073; // RISC-V Unprivileged ISA.

struct Probe(Vec<u32>);

impl Probe {
    fn li(&mut self, dest: u32, value: u32) {
        // RISC-V LUI/ADDI, младшие 12 бит знаковые.
        let high = value.wrapping_add(0x800) & !0xfff;
        self.0.extend([
            high | dest << 7 | 0x37,
            (value & 0xfff) << 20 | dest << 15 | dest << 7 | 0x13,
        ]);
    }

    fn put(&mut self, offset: u32, value: u32) {
        self.li(6, value);
        // sw t1,offset(t0).
        self.0
            .push((offset >> 5) << 25 | 0x0062_a023 | (offset & 31) << 7);
    }

    fn put_absolute(&mut self, addr: u32, value: u32) {
        self.li(5, addr);
        self.put(0, value);
    }

    fn raw_board(&self) -> Box<dyn BoardSim> {
        // TRM §3.3.2 (ROM) и efuse_reg.h (блоки readback).
        const ROM_LEN: usize = 384 * 1024;
        const EFUSE_WORDS: usize = (0x17c - 0x2c) / size_of::<u32>();
        let mut rom: Vec<_> = self.0.iter().flat_map(|word| word.to_le_bytes()).collect();
        rom.resize(ROM_LEN, 0);
        let mut board = new_board(BoardId::Esp32C3);
        board
            .load_esp_flash(
                include_bytes!("data/esp_peripherals/flash.bin"),
                esp_image::DEFAULT_PARTITION_TABLE_OFFSET,
                &rom,
                &[0; EFUSE_WORDS],
                [true; 3],
            )
            .unwrap();
        board
    }

    fn load(&mut self, offset: u32) {
        self.0.push(offset << 20 | 0x0002_a383); // lw t2,offset(t0).
    }

    fn expect(&mut self, offset: u32, value: u32) {
        self.load(offset);
        self.li(6, value);
        self.0.extend([0x0063_8463, EBREAK]); // beq t2,t1,+8.
    }

    fn wait_irq(&mut self, bit: u32) {
        self.load(MASKED);
        self.0.push(bit << 20 | 0x0003_f393); // andi t2,t2,bit.
        self.0.push(0xfe03_8ce3); // beq t2,x0,−8.
    }

    fn master() -> Self {
        let mut probe = Self(Vec::new());
        probe.li(5, BASE);
        // Штатный ESP-IDF v6.1.1, i2c_ll_master_init и i2c_ll_master_set_bus_timing:
        // XTAL 40 МГц, 400 кГц, half_cycle=50, WAIT_HIGH=23, HIGH=27.
        for (offset, value) in [
            (CTR, MASTER),
            (0x18, 0x708b), // FIFO_CONF: TX/RX reset, protect и reset watermarks.
            (0x00, 49),     // SCL_LOW_PERIOD.
            (0x38, 23 << 9 | 27),
            (0x30, 11), // SDA_HOLD = half_cycle/4−1.
            (0x34, 24), // SDA_SAMPLE = half_cycle/2−1.
            (0x40, 49),
            (0x44, 49),
            (0x48, 49),
            (0x4c, 49),          // SCL START/STOP.
            (0x50, 0x377),       // FILTER_CFG: пример SDK, порог 7 для SCL и SDA.
            (0x54, 0x0020_0000), // CLK_CONF: XTAL, SCLK_ACTIVE, целый делитель /1.
            (TO, (1 << 5) | 13),
            (CLR, u32::MAX),
            (ENA, DRIVER_EVENTS),
        ] {
            probe.put(offset, value);
        }
        probe
    }

    fn sequence(&mut self, commands: &[u32], bytes: &[u8]) {
        for &byte in bytes {
            self.put(FIFO, u32::from(byte));
        }
        for (i, &command) in commands.iter().enumerate() {
            self.put(COMMAND + 4 * i as u32, command);
        }
        self.put(CTR, MASTER | CONFIG_UPDATE);
        self.put(CTR, MASTER_START);
    }

    fn board(mut self) -> (Box<dyn BoardSim>, u32) {
        let tail = ENTRY + self.0.len() as u32 * 4;
        self.0.push(EBREAK);
        // ELF32 LE, System V ABI: один четырёхбайтно выровненный сегмент PT_LOAD.
        let mut elf = vec![0; 52 + 32];
        elf[..7].copy_from_slice(b"\x7fELF\x01\x01\x01");
        elf[16..20].copy_from_slice(&[2, 0, 243, 0]); // ET_EXEC, EM_RISCV.
        let bytes = self.0.len() as u32 * 4;
        for (offset, value) in [
            (20, 1),
            (24, ENTRY),
            (28, 52),
            (40, 32 << 16 | 52),
            (44, 1),
            (52, 1),
            (56, 84),
            (60, ENTRY),
            (64, ENTRY),
            (68, bytes),
            (72, bytes),
            (76, 5),
            (80, 4),
        ] {
            elf[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
        }
        elf.extend(self.0.into_iter().flat_map(u32::to_le_bytes));
        let mut board = new_board(BoardId::Esp32C3);
        board.load(&elf).unwrap();
        (board, tail)
    }
}

fn request(board: &mut dyn BoardSim) -> (u64, I2cRequest) {
    assert!(board.run_cycles(100_000).unwrap().bus_request_pending);
    let [event] = board.take_events().try_into().unwrap();
    let BoardOutput::I2c(request) = event.output else {
        panic!("ожидался запрос I²C, получен {event:?}");
    };
    (event.cycle, request)
}

fn advance(board: &mut dyn BoardSim, cycle: u64) {
    assert_eq!(
        board.run_cycles(cycle - board.cycles()).unwrap().cycle,
        cycle
    );
}

fn reply(
    board: &mut dyn BoardSim,
    id: u64,
    status: TransferStatus,
    data: &[u8],
) -> Result<InputOutcome, BoardFault> {
    board.apply(InputEvent {
        cycle: board.cycles(),
        input: BoardInput::I2cReply {
            bus: 0,
            id,
            status,
            bytes: data.to_vec(),
        },
    })
}

fn done(
    board: &mut dyn BoardSim,
    request: &I2cRequest,
    cycle: u64,
    status: TransferStatus,
    data: &[u8],
) {
    assert_eq!(
        board.take_events(),
        vec![OutputEvent {
            cycle,
            output: BoardOutput::I2cDone {
                bus: 0,
                id: request.id,
                address: ADDRESS,
                status,
                bytes: data.to_vec()
            }
        }]
    );
}

fn reached_end(board: &mut dyn BoardSim, pc: u32) {
    assert_eq!(
        board.run_cycles(30_000).unwrap_err(),
        BoardFault::Breakpoint { pc }
    );
}

#[test]
fn i2c_совмещённая_запись_чтение_с_растяжением() {
    let mut probe = Probe::master();
    probe.sequence(
        &[
            RESTART,
            WRITE_ACK | 2,
            RESTART,
            WRITE_ACK | 1,
            READ_ACK | 1,
            READ_NACK | 1,
            STOP,
        ],
        &[ADDRESS << 1, 0x55, ADDRESS << 1 | 1],
    );
    probe.wait_irq(COMPLETE);
    probe.expect(RAW, COMPLETE);
    probe.expect(COMMAND + 24, STOP | DONE);
    probe.expect(SR, 2 << 8);
    probe.expect(FIFO, 0xa5);
    probe.expect(FIFO, 0x5a);
    probe.put(ENA, 0);
    probe.expect(MASKED, 0);
    probe.expect(RAW, COMPLETE);
    probe.put(CLR, COMPLETE);
    probe.expect(RAW, 0);
    let (mut board, pc) = probe.board();
    let (at, req) = request(&mut *board);
    assert_eq!(
        (req.address, req.cycles_per_bit, req.read_len),
        (ADDRESS, FAST_BIT, 2)
    );
    assert_eq!(req.write, [0x55]);
    assert!(req.repeated_start);
    // TRM §28.5.4: START=0.5 бит, RSTART=1 бит, пять байтов по девять бит, STOP=1 бит.
    assert_eq!(req.address_ack_cycle - at, FAST_BIT / 2 + 9 * FAST_BIT);
    assert_eq!(
        req.complete_cycle - at,
        FAST_BIT / 2 + (45 + 1 + 1) * FAST_BIT
    );
    let stretch = FAST_BIT * 3; // Стенд удерживает SCL три битовых интервала.
    advance(&mut *board, req.address_ack_cycle + stretch);
    assert!(
        reply(
            &mut *board,
            req.id + 1,
            TransferStatus::Complete,
            &[0xa5, 0x5a]
        )
        .is_err()
    );
    assert!(reply(&mut *board, req.id, TransferStatus::Complete, &[0xa5]).is_err());
    reply(&mut *board, req.id, TransferStatus::Complete, &[0xa5, 0x5a]).unwrap();
    assert!(reply(&mut *board, req.id, TransferStatus::Complete, &[0, 0]).is_err());
    let end = req.complete_cycle + stretch;
    let mut clone = board.clone_state();
    for board in [&mut *board, &mut *clone] {
        advance(board, end - 1);
        assert!(board.take_events().is_empty());
        advance(board, end);
        done(board, &req, end, TransferStatus::Complete, &[0xa5, 0x5a]);
        reached_end(board, pc);
    }
}

#[test]
fn i2c_просрочка_даёт_только_timeout_и_сброс_возвращает_работу() {
    let mut probe = Probe::master();
    let cmd = [RESTART, WRITE_ACK | 1, STOP];
    probe.sequence(&cmd, &[ADDRESS << 1]);
    probe.wait_irq(TIMEOUT);
    probe.expect(RAW, TIMEOUT);
    probe.expect(COMMAND, RESTART); // Контроллер не подтверждает исполнение пакета.
    probe.expect(SR, 0);
    probe.put(CLR, TIMEOUT);
    probe.put(CTR, MASTER | (1 << 10)); // I2C_FSM_RST.
    probe.sequence(&cmd, &[ADDRESS << 1]);
    probe.wait_irq(COMPLETE);
    probe.expect(MASKED, COMPLETE);
    let (mut board, pc) = probe.board();
    let (_, first) = request(&mut *board);
    let deadline = first.address_ack_cycle + STRETCH_LIMIT;
    let mut held = board.clone_state();
    reply(&mut *held, first.id, TransferStatus::NoResponse, &[]).unwrap();
    for branch in [&mut *board, &mut *held] {
        advance(branch, deadline - 1);
        assert!(branch.take_events().is_empty());
        advance(branch, deadline);
        done(branch, &first, deadline, TransferStatus::NoResponse, &[]);
        assert!(reply(branch, first.id, TransferStatus::Complete, &[]).is_err());
        let (_, second) = request(branch);
        assert_ne!(first.id, second.id);
        assert!(reply(branch, first.id, TransferStatus::Complete, &[]).is_err());
        reply(branch, second.id, TransferStatus::Complete, &[]).unwrap();
        advance(branch, second.complete_cycle);
        done(
            branch,
            &second,
            second.complete_cycle,
            TransferStatus::Complete,
            &[],
        );
        reached_end(branch, pc);
    }
}

#[test]
fn i2c_nack_в_адресе_не_доставляет_данные() {
    let mut probe = Probe::master();
    probe.sequence(
        &[RESTART, WRITE_ACK | 1, READ_NACK | 1, STOP],
        &[ADDRESS << 1 | 1],
    );
    probe.wait_irq(NACK);
    probe.expect(MASKED, NACK);
    probe.expect(SR, BUSY | 1);
    probe.expect(COMMAND + 4, WRITE_ACK | 1);
    probe.put(CLR, NACK);
    probe.wait_irq(COMPLETE);
    probe.expect(SR, 1); // RESP_REC — NACK, BUS_BUSY снят сигналом STOP.
    let (mut board, pc) = probe.board();
    let (_, req) = request(&mut *board);
    assert!(reply(&mut *board, req.id, TransferStatus::Nack, &[0]).is_err());
    reply(&mut *board, req.id, TransferStatus::Nack, &[]).unwrap();
    advance(&mut *board, req.address_ack_cycle - 1);
    assert!(board.take_events().is_empty());
    advance(&mut *board, req.address_ack_cycle);
    done(
        &mut *board,
        &req,
        req.address_ack_cycle,
        TransferStatus::Nack,
        &[],
    );
    reached_end(&mut *board, pc);
}

#[test]
fn i2c_end_оставляет_шину_занятой_между_порциями_fifo() {
    let mut probe = Probe::master();
    probe.sequence(
        &[RESTART, WRITE_ACK | 1, READ_ACK | 32, END],
        &[ADDRESS << 1 | 1],
    );
    probe.wait_irq(END_DETECT);
    probe.expect(MASKED, END_DETECT);
    probe.expect(SR, BUSY | (32 << 8));
    for byte in 0..32 {
        probe.expect(FIFO, byte);
    }
    probe.put(CLR, END_DETECT);
    probe.sequence(&[READ_NACK | 1, STOP], &[]);
    probe.wait_irq(COMPLETE);
    probe.expect(FIFO, 32);
    probe.expect(SR, 0);
    let (mut board, pc) = probe.board();
    let (at, first) = request(&mut *board);
    let data: Vec<u8> = (0..32).collect();
    assert_eq!(first.read_len, data.len());
    assert_eq!(first.complete_cycle - at, FAST_BIT / 2 + 33 * 9 * FAST_BIT);
    reply(&mut *board, first.id, TransferStatus::Complete, &data).unwrap();
    advance(&mut *board, first.complete_cycle);
    done(
        &mut *board,
        &first,
        first.complete_cycle,
        TransferStatus::Complete,
        &data,
    );
    let (at, second) = request(&mut *board);
    assert_ne!(first.id, second.id);
    assert_eq!(
        (second.address, second.read_len, second.repeated_start),
        (ADDRESS, 1, false)
    );
    assert_eq!(second.complete_cycle - at, (9 + 1) * FAST_BIT);
    reply(&mut *board, second.id, TransferStatus::Complete, &[32]).unwrap();
    advance(&mut *board, second.complete_cycle);
    done(
        &mut *board,
        &second,
        second.complete_cycle,
        TransferStatus::Complete,
        &[32],
    );
    reached_end(&mut *board, pc);
}

#[test]
fn i2c_nack_перед_end_требует_отдельного_stop() {
    let mut probe = Probe::master();
    probe.sequence(&[RESTART, WRITE_ACK | 2, END], &[ADDRESS << 1, 0x55]);
    probe.wait_irq(NACK);
    probe.expect(RAW, NACK);
    probe.expect(SR, BUSY | 1);
    probe.put(CLR, NACK);
    // i2c_master.c::s_i2c_send_commands запускает восстановительный STOP
    // внутри ветки ACK_ERROR и повторяет запуск после выхода из цикла.
    probe.sequence(&[STOP], &[]);
    probe.put(CTR, MASTER_START);
    probe.wait_irq(COMPLETE);
    probe.expect(RAW, COMPLETE);
    probe.expect(SR, 0);
    let (mut board, pc) = probe.board();
    let (_, req) = request(&mut *board);
    reply(&mut *board, req.id, TransferStatus::Nack, &[]).unwrap();
    advance(&mut *board, req.address_ack_cycle);
    done(
        &mut *board,
        &req,
        req.address_ack_cycle,
        TransferStatus::Nack,
        &[],
    );
    reached_end(&mut *board, pc);
}

#[test]
fn i2c_неизвестный_режим_и_неполная_команда_явно_отклоняются() {
    for (commands, bytes, unsupported_expected) in [
        (vec![RESTART, WRITE_ACK | 1, STOP], vec![0xf0], true), // 10-битный префикс.
        (
            vec![RESTART, WRITE_ACK | 2, STOP],
            vec![ADDRESS << 1],
            false,
        ), // Не хватает байта TX.
        (
            vec![RESTART, WRITE_ACK | 1, READ_ACK | 1, STOP],
            vec![ADDRESS << 1 | 1],
            false,
        ),
    ] {
        let mut probe = Probe::master();
        probe.sequence(&commands, &bytes);
        let (mut board, _) = probe.board();
        let error = board.run_cycles(10_000).unwrap_err();
        if unsupported_expected {
            assert!(matches!(error, BoardFault::Unsupported(_)), "{error:?}");
        } else {
            assert!(matches!(error, BoardFault::Misconfigured(_)), "{error:?}");
        }
        assert!(
            !board
                .take_events()
                .iter()
                .any(|event| matches!(event.output, BoardOutput::I2c(_)))
        );
    }
}

#[test]
fn uart_raw_flash_маршрут_кадр_irq_и_tx() {
    // reg_base.h, gpio_reg.h, io_mux_reg.h и gpio_sig_map.h.
    const UART: u32 = 0x6000_0000;
    const ROM: u32 = 0x4000_0000;
    let mut p = Probe(Vec::new());
    p.put_absolute(0x6000_9054, 0); // Прямой GPIO20, FUN_IE снят.
    let disconnected = p.0.len() as u64;
    p.put_absolute(0x6000_9014, 0x1200); // GPIO4: FUN_IE и MCU_SEL=1.
    p.put_absolute(0x6000_416c, 0x40 | 4); // U0RXD_IN_IDX=6, SIG_IN_SEL, GPIO4.
    p.put_absolute(0x6000_9058, 0); // FUNC_U0TXD_U0TXD=0.
    p.li(5, UART);
    for (offset, value) in [
        (0x14, 0x0030_015b),
        (0x78, 0x0370_0000),
        (0x48, 0x100),
        (0x24, 0x0020_0002),
        (UART_ENABLE, UART_FULL | UART_TIMEOUT | UART_TX_DONE),
    ] {
        p.put(offset, value);
    }
    let configured = p.0.len() as u64;
    // wait_irq использует I²C_MASKED, UART опрашивается по UART_INT_ST.
    p.load(UART_MASKED);
    p.0.push(0xfe03_8ee3); // beq t2,x0,−4
    p.expect(UART_MASKED, UART_FULL);
    for byte in b"ABC" {
        p.expect(0, u32::from(*byte));
    }
    p.expect(UART_MASKED, 0);
    p.put(0, 0x5a);
    let tx = p.0.len() as u64 - 1;
    // Тот же точный целочисленный расчёт независимо от частоты CPU:
    // ROM стартует от XTAL/2, 10×(5555/16) периодов XTAL дают 1736 тактов CPU.
    let delay = (10_u64 * 5555).div_ceil(32);
    p.0.resize((tx + delay + 1) as usize, 0x0000_0013); // NOP.
    p.expect(UART_MASKED, UART_TX_DONE);
    p.put(UART_CLEAR, UART_TX_DONE);
    p.expect(UART_MASKED, 0);
    let end = ROM + p.0.len() as u32 * 4;
    p.0.push(EBREAK);
    let mut board = p.raw_board();
    let input = |cycle| InputEvent {
        cycle,
        input: BoardInput::Uart {
            port: 0,
            bytes: b"ABC".to_vec(),
        },
    };
    advance(&mut *board, disconnected);
    assert!(
        matches!(board.apply(input(disconnected)), Err(BoardFault::Misconfigured(s)) if s.contains("маршрут"))
    );
    advance(&mut *board, configured);
    assert_eq!(
        board.apply(input(configured)),
        Ok(InputOutcome::Uart {
            accepted: 3,
            dropped: 0
        })
    );
    let third = configured + (30_u64 * 5555).div_ceil(32);
    let mut copy = board.clone_state();
    for board in [&mut board, &mut copy] {
        advance(&mut **board, third - 1);
        let pos = board.esp_boot_position().unwrap();
        assert!(
            [configured, configured + 1]
                .map(|n| ROM + n as u32 * 4)
                .contains(&pos.pc),
            "{pos:?}"
        );
        assert!(board.take_events().is_empty());
        advance(&mut **board, third);
        let tx_pc = ROM + tx as u32 * 4;
        for _ in configured..=tx {
            if board.esp_boot_position().unwrap().pc == tx_pc {
                break;
            }
            board.run_cycles(1).unwrap();
        }
        assert_eq!(board.esp_boot_position().unwrap().pc, tx_pc);
        let tx_done = board.cycles() + delay;
        advance(&mut **board, tx_done - 1);
        let early = board.take_events();
        assert!(early.is_empty(), "до срока {tx_done}: {early:?}");
        advance(&mut **board, tx_done);
        let events = board.take_events();
        let [event] = events.as_slice() else {
            panic!("{events:?}")
        };
        assert_eq!(
            event.output,
            BoardOutput::Uart {
                port: 0,
                byte: b'Z'
            }
        );
        // Цикл запуска зависит от двухтактного опроса IRQ.
        assert_eq!(event.cycle, tx_done);
        reached_end(&mut **board, end);
    }
}

#[test]
fn i2c_raw_flash_маршруты_nack_timeout_и_часы() {
    for (routed, send_nack) in [(false, false), (true, true), (true, false)] {
        let mut p = Probe(Vec::new());
        // system_reg.h SYSTEM_I2C_EXT0_CLK_EN, остальные поля соответствуют сбросу.
        p.put_absolute(0x600c_0010, 0xf9c1_e06f | 1 << 7);
        if routed {
            // GPIO4/5, I2CEXT0_{SCL,SDA}_{IN,OUT}_IDX=53/54, open drain.
            for (pin, signal) in [(4, 53), (5, 54)] {
                p.put_absolute(0x6000_9004 + 4 * pin, 0x1300);
                p.put_absolute(0x6000_4074 + 4 * pin, 4);
                p.put_absolute(0x6000_4154 + 4 * signal, 0x40 | pin);
                p.put_absolute(0x6000_4554 + 4 * pin, signal);
            }
        }
        p.0.extend(Probe::master().0);
        p.sequence(&[RESTART, WRITE_ACK | 1, STOP], &[ADDRESS << 1]);
        if send_nack {
            p.wait_irq(NACK);
            p.expect(RAW, NACK);
            p.wait_irq(COMPLETE);
            p.expect(RAW, NACK | COMPLETE);
        } else {
            p.wait_irq(TIMEOUT);
            p.expect(RAW, TIMEOUT);
        }
        let pc = 0x4000_0000 + p.0.len() as u32 * 4; // TRM §3.3.2, reset ROM.
        p.0.push(EBREAK);
        let mut board = p.raw_board();
        if !routed {
            assert!(
                matches!(board.run_cycles(1000), Err(BoardFault::Misconfigured(s))
                if s.contains("входного маршрута"))
            );
            assert!(
                board
                    .take_events()
                    .iter()
                    .all(|event| !matches!(event.output, BoardOutput::I2c(_)))
            );
            continue;
        }
        let (at, req) = request(&mut *board);
        // 20 МГц CPU/400 кГц SCL; START и каждый интервал STOP по 50 тактов XTAL.
        assert_eq!(req.cycles_per_bit, 50);
        assert_eq!(req.address_ack_cycle - at, 25 + 9 * 50);
        assert_eq!(req.complete_cycle - at, 25 + 9 * 50 + 2 * 25);
        if send_nack {
            assert_eq!(
                reply(&mut *board, req.id, TransferStatus::Nack, &[]),
                Ok(InputOutcome::Applied)
            );
            advance(&mut *board, req.address_ack_cycle);
            done(
                &mut *board,
                &req,
                req.address_ack_cycle,
                TransferStatus::Nack,
                &[],
            );
        } else {
            // I2C_TO: 2^13 тактов XTAL после границы ACK, CPU=XTAL/2.
            let deadline = req.address_ack_cycle + (1 << 12);
            advance(&mut *board, deadline - 1);
            assert!(board.take_events().is_empty());
            advance(&mut *board, deadline);
            assert!(reply(&mut *board, req.id, TransferStatus::Complete, &[]).is_err());
            done(&mut *board, &req, deadline, TransferStatus::NoResponse, &[]);
        }
        reached_end(&mut *board, pc);
    }
}

#[test]
fn uart_raw_flash_кадр_от_apb_переживает_смену_делителя_cpu() {
    // TRM §6.2.4.1 и uart_reg.h: APB=CPU при XTAL, CLK_CONF SCLK_SEL=1.
    const UART: u32 = 0x6000_0000;
    const SYSCLK: u32 = 0x600c_0058;
    const ROM: u32 = 0x4000_0000;
    let mut p = Probe(Vec::new());
    p.li(5, UART);
    p.put(0x14, 0x0030_015b);
    p.put(0x78, 0x0350_0000);
    p.put(0x48, 0x100);
    p.put(0, u32::from(b'Z'));
    let start = p.0.len() as u64 - 1;
    p.put_absolute(SYSCLK, 0); // XTAL/2 → XTAL/1, APB вслед за CPU.
    let switch = p.0.len() as u64;
    let finish = start + (10_u64 * 5555).div_ceil(16);
    p.0.resize(finish as usize, 0x0000_0013); // RISC-V NOP до конца кадра.
    p.0.push(EBREAK);
    let mut board = p.raw_board();
    advance(&mut *board, finish - 1);
    assert!(board.take_events().is_empty());
    advance(&mut *board, finish);
    assert_eq!(
        board.take_events(),
        vec![OutputEvent {
            cycle: finish,
            output: BoardOutput::Uart {
                port: 0,
                byte: b'Z'
            }
        }]
    );
    // До смены 200 четвертей нс на такт (20 МГц), после — 100 (40 МГц).
    assert_eq!(
        board.cycle_time_quarter_ns(finish).unwrap().unwrap()
            - board.cycle_time_quarter_ns(start).unwrap().unwrap(),
        u128::from((switch - start) * 200 + (finish - switch) * 100)
    );
    reached_end(&mut *board, ROM + finish as u32 * 4);
}

#[test]
fn uart_raw_flash_уровневый_irq_входит_в_вектор_только_с_маской() {
    // TRM §1.4.2 и interrupt_core0_reg.h: вектор и матрица, UART0 — источник 21.
    const ROM: u32 = 0x4000_0000;
    const IRQ: u32 = 0x600c_2000;
    const UART: u32 = 0x6000_0000;
    const LINE: u32 = 5; // Выбор линии на стенде.
    const VECTOR: u32 = ROM + 0x400;
    const HANDLER: u32 = VECTOR + LINE * 4;
    const STOP_CSR: u32 = 0xfff; // Неизвестный CSR останавливает модель вне гостевых ловушек.
    const STOP: u32 = 0xfff0_22f3;
    for enabled in [false, true] {
        let mut p = Probe(Vec::new());
        p.li(5, VECTOR | 1);
        p.0.push(0x3052_9073); // csrw mtvec,t0.
        p.put_absolute(IRQ + 21 * 4, LINE);
        p.put_absolute(IRQ + 0x114 + LINE * 4, 1); // CPU_INT_PRI_5.
        p.put_absolute(IRQ + 0x104, u32::from(enabled) << LINE);
        p.li(5, UART);
        p.put(0x14, 16); // UART_CLKDIV: 16 тактов XTAL на бит.
        p.put(0x78, 0x0370_0000);
        p.put(0x24, 0); // RXFIFO_FULL_THRHD=0.
        p.put(UART_ENABLE, UART_FULL);
        let ready = p.0.len() as u64;
        p.0.push(0x3004_6073); // csrsi mstatus,8
        p.load(UART_MASKED);
        p.0.push(0xfe03_8ee3); // beq t2,x0,−4
        let main = ROM + p.0.len() as u32 * 4;
        p.0.push(STOP);
        p.0.resize(((HANDLER - ROM) / 4) as usize, 0);
        p.0.push(0x3420_23f3); // csrr t2,mcause.
        p.li(6, (1 << 31) | LINE);
        p.0.extend([0x0063_8463, STOP]); // beq t2,t1,+8.
        let handler = ROM + p.0.len() as u32 * 4;
        p.0.push(STOP);
        let mut board = p.raw_board();
        advance(&mut *board, ready);
        assert_eq!(
            board.apply(InputEvent {
                cycle: ready,
                input: BoardInput::Uart {
                    port: 0,
                    bytes: vec![b'Q']
                }
            }),
            Ok(InputOutcome::Uart {
                accepted: 1,
                dropped: 0
            })
        );
        // 10 бит × /16 XTAL × CPU/XTAL = 80 тактов до UART_FULL.
        advance(&mut *board, ready + 79);
        assert_eq!(
            board.run_cycles(40),
            Err(BoardFault::UnsupportedRegister {
                pc: if enabled { handler } else { main },
                addr: STOP_CSR,
                write: false,
                size: 4,
                block: "CSR RV32",
            })
        );
    }
}
