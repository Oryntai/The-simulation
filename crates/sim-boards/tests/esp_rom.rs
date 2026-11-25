//! Сверка эффектов ROM с QEMU Espressif 9.2.2. В QEMU стенд подменяет PC
//! на 0x40380000 на штатном входе второй ступени, вызывает те же адреса
//! через JALR и читает результаты из SRAM1 через GDB. Сборка ESP-IDF v6.1.1.

use std::num::NonZeroU64;

use esp_memory::EspMemory;
use sim_boards::{BoardFault, esp_image, esp_memory};

// Как в Т2.5, стенд проверяет модуль до подключения к шине в Т2.10.
#[allow(dead_code)]
#[path = "../src/peripheral/esp_flash.rs"]
mod esp_flash;
#[allow(dead_code)]
#[path = "../src/esp_rom.rs"]
mod esp_rom;

use esp_flash::EspFlash;
use esp_rom::calls::{FlashRead, RomCall, RomIo, RomStep, UartTx, UartWait, dispatch};

// Адреса функций из esp_rom/esp32c3/ld/esp32c3.rom*.ld, ESP-IDF v6.1.1.
const SPI_READ: u32 = 0x4000_0130;
const MEMSET: u32 = 0x4000_0354;
const MEMCPY: u32 = 0x4000_0358;
const MEMMOVE: u32 = 0x4000_035c;
const MEMCMP: u32 = 0x4000_0360;
const CRC32_LE: u32 = 0x4000_0628;
const UART_TX: u32 = 0x4000_0068;
const UART_RX: u32 = 0x4000_0070;
const UART_FLUSH: u32 = 0x4000_0080;
const UART_IDLE: u32 = 0x4000_0084;
const DELAY_US: u32 = 0x4000_0050;
const GET_TICKS: u32 = 0x4000_0584;
const SET_TICKS: u32 = 0x4000_0588;
// TRM §3.3.2: окно SRAM1. Смещения в нём выбраны для стенда.
const RETURN_PC: u32 = 0x4038_000c;
const BASE: u32 = 0x3fc8_1000;
const DATA: u32 = BASE + 64;
const VECTOR: u32 = BASE + 128;
const END_OF_SRAM: u32 = 0x3fce_0000;
// tools/tools.json (esp-rom-elfs), файл esp32c3_rev3_rom.elf: s_ticks_per_us.
const TICKS: u32 = 0x3fcd_f64c;
// tools/idf_py_actions/qemu_ext.py, штатный raw flash QEMU и раздел bootloader.
const TABLE: u32 = esp_image::DEFAULT_PARTITION_TABLE_OFFSET;
const CAPACITY: u32 = 2 << 20;
// TRM §3.3.3 и esp32c3/include/soc/ext_mem_defs.h, SOC_DRAM0_CACHE_ADDRESS_LOW.
const DROM: u32 = 0x3c00_0000;
const PARTITION_BYTES: [u8; 8] = [0xaa, 0x50, 0x01, 0x02, 0x00, 0x90, 0x00, 0x00];

#[derive(Default)]
struct Io {
    flash: Option<EspFlash>,
    flash_override: Option<FlashRead<'static>>,
    flash_request: Option<(u32, usize)>,
    tx: Vec<u8>,
    tx_busy: Option<NonZeroU64>,
    tx_error: bool,
    rx: Option<u8>,
    uart_state: Option<(u8, UartWait)>,
    uart_busy: Option<NonZeroU64>,
    serial_fault: Option<BoardFault>,
}

impl RomIo for Io {
    fn read_flash(&mut self, addr: u32, len: usize) -> Result<FlashRead<'_>, BoardFault> {
        self.flash_request = Some((addr, len));
        if let Some(response) = self.flash_override {
            return Ok(response);
        }
        let flash = self
            .flash
            .as_ref()
            .ok_or_else(|| BoardFault::Unsupported("нет снимка flash стенда".into()))?;
        match flash.read_physical(addr, len) {
            Ok(bytes) => Ok(FlashRead::Bytes(bytes)),
            Err(BoardFault::BadImage(_)) => Ok(FlashRead::Error),
            Err(other) => Err(other),
        }
    }

    fn read_cached_byte(&mut self, pc: u32, addr: u32) -> Result<u8, BoardFault> {
        let flash = self
            .flash
            .as_mut()
            .ok_or_else(|| BoardFault::Unsupported("нет снимка flash стенда".into()))?;
        let byte = flash
            .access(pc, addr, 1, None)?
            .ok_or(BoardFault::BusError {
                pc,
                addr,
                write: false,
            })?;
        Ok(byte as u8)
    }

    fn uart_send_console(&mut self, byte: u8) -> Result<UartTx, BoardFault> {
        if let Some(error) = &self.serial_fault {
            return Err(error.clone());
        }
        if let Some(cycles) = self.tx_busy {
            return Ok(UartTx::Busy(cycles));
        }
        if self.tx_error {
            return Ok(UartTx::Error);
        }
        self.tx.push(byte);
        Ok(UartTx::Sent)
    }

    fn uart_receive_console(&mut self) -> Result<Option<u8>, BoardFault> {
        Ok(self.rx.take())
    }

    fn uart_wait(
        &mut self,
        serial: u8,
        condition: UartWait,
    ) -> Result<Option<NonZeroU64>, BoardFault> {
        self.uart_state = Some((serial, condition));
        Ok(self.uart_busy)
    }
}

fn call(
    entry: u32,
    args: [u32; 3],
    memory: &mut EspMemory,
    io: &mut Io,
) -> Result<RomStep, BoardFault> {
    // RISC-V psABI: аргументы a0–a2, адрес возврата ra. JALR очищает bit 0.
    dispatch(
        RomCall {
            entry,
            ra: RETURN_PC | 1,
            args,
        },
        memory,
        io,
    )
}

fn returned(value: Option<u32>, hold_cycles: u64) -> RomStep {
    RomStep::Return {
        pc: RETURN_PC,
        value,
        hold_cycles,
    }
}

fn load(memory: &mut EspMemory, base: u32, bytes: &[u8]) {
    for (offset, &byte) in bytes.iter().enumerate() {
        memory
            .write(RETURN_PC, base + offset as u32, 1, u32::from(byte))
            .unwrap();
    }
}

fn read(memory: &mut EspMemory, base: u32, len: u32) -> Vec<u8> {
    (base..base + len)
        .map(|addr| memory.read(RETURN_PC, addr, 1).unwrap() as u8)
        .collect()
}

#[test]
fn crc_в_обоих_порядках_совпадает_с_qemu() {
    let mut memory = EspMemory::default();
    let mut io = Io::default();
    load(&mut memory, VECTOR, b"123456789");
    // Слова a0 в QEMU. Для CRC32 LE дополнительно: CRC-32/ISO-HDLC
    // с нулевым seed у интерфейса ESP-IDF имеет check=0xcbf43926.
    for (pc, expected) in [
        (CRC32_LE, 0xcbf4_3926),
        (0x4000_062c, 0xfc89_1918),
        (0x4000_0630, 0x906e),
        (0x4000_0634, 0xd64e),
        (0x4000_0638, 0x2f),
        (0x4000_063c, 0x04),
    ] {
        assert_eq!(
            call(pc, [0, VECTOR, 9], &mut memory, &mut io),
            Ok(returned(Some(expected), 0))
        );
    }
    let RomStep::Return {
        value: Some(part), ..
    } = call(CRC32_LE, [0x1020_3040, VECTOR, 4], &mut memory, &mut io).unwrap()
    else {
        panic!("CRC не вернула слово a0");
    };
    assert_eq!(
        call(CRC32_LE, [part, VECTOR + 4, 5], &mut memory, &mut io),
        Ok(returned(Some(0x5cc0_525b), 0))
    );
    assert_eq!(
        call(CRC32_LE, [42, u32::MAX, 0], &mut memory, &mut io),
        Ok(returned(Some(42), 0))
    );
    assert_eq!(
        call(CRC32_LE, [0, END_OF_SRAM - 1, 2], &mut memory, &mut io),
        Err(BoardFault::BusError {
            pc: CRC32_LE,
            addr: END_OF_SRAM,
            write: false
        })
    );
}

#[test]
fn libc_возвращает_a0_и_меняет_ровно_заданные_байты() {
    let mut memory = EspMemory::default();
    let mut io = Io::default();
    load(&mut memory, VECTOR, b"123456789");
    load(&mut memory, DATA, &[0; 12]);
    assert_eq!(
        call(MEMSET, [DATA + 1, 0x12_345a, 5], &mut memory, &mut io),
        Ok(returned(Some(DATA + 1), 0))
    );
    assert_eq!(read(&mut memory, DATA, 8), b"\0ZZZZZ\0\0");
    assert_eq!(
        call(MEMCPY, [DATA + 3, VECTOR + 1, 5], &mut memory, &mut io),
        Ok(returned(Some(DATA + 3), 0))
    );
    assert_eq!(
        call(MEMMOVE, [DATA + 4, DATA + 3, 5], &mut memory, &mut io),
        Ok(returned(Some(DATA + 4), 0))
    );
    // Три слова SRAM QEMU после последовательности выше.
    let words = [0x325a_5a00u32, 0x3534_3332, 0x0000_0036];
    let expected = words
        .into_iter()
        .flat_map(u32::to_le_bytes)
        .collect::<Vec<_>>();
    assert_eq!(read(&mut memory, DATA, 12), expected);
    assert_eq!(
        call(MEMCMP, [DATA + 3, VECTOR + 1, 2], &mut memory, &mut io),
        Ok(returned(Some((-1i32) as u32), 0))
    );
    assert_eq!(
        call(MEMCMP, [DATA + 4, VECTOR + 1, 5], &mut memory, &mut io),
        Ok(returned(Some(0), 0))
    );
    assert!(matches!(
        call(MEMCPY, [DATA + 4, DATA + 3, 5], &mut memory, &mut io),
        Err(BoardFault::Misconfigured(_))
    ));
    // Неподготовленный последний байт SRAM не должен оставить частичную запись.
    assert!(matches!(
        call(MEMCPY, [DATA, VECTOR + 8, 2], &mut memory, &mut io),
        Err(BoardFault::Unsupported(_))
    ));
    assert_eq!(read(&mut memory, DATA, 12), expected);
    let end = END_OF_SRAM - 1;
    load(&mut memory, end, b"\xa5");
    assert_eq!(
        call(MEMSET, [end, 0, 2], &mut memory, &mut io),
        Err(BoardFault::BusError {
            pc: MEMSET,
            addr: END_OF_SRAM,
            write: true
        })
    );
    assert_eq!(read(&mut memory, end, 1), b"\xa5");
}

#[test]
fn flash_сверяется_с_таблицей_qemu_и_передаёт_ошибки() {
    let mut memory = EspMemory::default();
    let mut raw = vec![0xff; TABLE as usize + PARTITION_BYTES.len()];
    raw[TABLE as usize..].copy_from_slice(&PARTITION_BYTES);
    let mut io = Io {
        flash: Some(EspFlash::new(&raw, CAPACITY).unwrap()),
        ..Io::default()
    };
    let dst = DATA + 16;
    load(&mut memory, dst - 1, &[0xa5; 10]);
    assert_eq!(
        call(SPI_READ, [TABLE, dst, 8], &mut memory, &mut io),
        Ok(returned(Some(0), 0))
    );
    assert_eq!(io.flash_request, Some((TABLE, 8)));
    assert_eq!(read(&mut memory, dst, 8), PARTITION_BYTES);
    assert_eq!(read(&mut memory, dst - 1, 1), [0xa5]);
    assert_eq!(read(&mut memory, dst + 8, 1), [0xa5]);
    assert_eq!(
        call(SPI_READ, [CAPACITY, dst, 4], &mut memory, &mut io),
        Ok(returned(Some(1), 0))
    );
    assert!(matches!(
        call(SPI_READ, [TABLE + 8, dst, 4], &mut memory, &mut io),
        Err(BoardFault::Unsupported(_))
    ));
    io.flash_override = Some(FlashRead::Timeout);
    assert_eq!(
        call(SPI_READ, [TABLE, dst, 8], &mut memory, &mut io),
        Ok(returned(Some(2), 0))
    );
    io.flash_override = Some(FlashRead::Bytes(&PARTITION_BYTES[..4]));
    assert!(matches!(
        call(SPI_READ, [TABLE, dst, 8], &mut memory, &mut io),
        Err(BoardFault::Misconfigured(_))
    ));
    assert_eq!(
        call(SPI_READ, [TABLE, dst + 1, 4], &mut memory, &mut io),
        Err(BoardFault::Unaligned {
            pc: SPI_READ,
            addr: dst + 1,
            size: 4
        })
    );
    assert!(matches!(
        call(SPI_READ, [TABLE, dst, u32::MAX], &mut memory, &mut io),
        Err(BoardFault::Misconfigured(_))
    ));
    assert_eq!(read(&mut memory, dst, 8), PARTITION_BYTES);
}

#[test]
fn чтение_drom_для_libc_и_crc_проходит_через_cache_и_mmu() {
    let mut memory = EspMemory::default();
    load(&mut memory, DATA, b"xxxxxxxxx");
    // Стенд на первой строке flash. Длина строки 32 байта, TRM §3.3.3.
    let mut raw = [0xff; 32];
    raw[..9].copy_from_slice(b"123456789");
    let mut io = Io {
        flash: Some(EspFlash::new(&raw, CAPACITY).unwrap()),
        ..Io::default()
    };
    assert_eq!(
        call(MEMCPY, [DATA, DROM, 9], &mut memory, &mut io),
        Err(BoardFault::BusError {
            pc: MEMCPY,
            addr: DROM,
            write: false
        })
    );
    assert_eq!(read(&mut memory, DATA, 9), b"xxxxxxxxx");

    // ESP-IDF esp32c3/register/soc/{sensitive_reg,spi_mem_reg,extmem_reg}.h:
    // SRAM_USAGE, SPI0_CACHE_FCTRL, DBUS/IBUS SHUT, MMU[0], ICACHE_ENABLE.
    let flash = io.flash.as_mut().unwrap();
    for (addr, value) in [
        (0x600c_101c, 0),
        (0x6000_303c, 1),
        (0x600c_4004, 0),
        (0x600c_5000, 0),
        (0x600c_4000, 1),
    ] {
        assert_eq!(flash.access(RETURN_PC, addr, 4, Some(value)), Ok(Some(0)));
    }
    assert_eq!(
        call(MEMCPY, [DATA, DROM, 9], &mut memory, &mut io),
        Ok(returned(Some(DATA), 0))
    );
    assert_eq!(read(&mut memory, DATA, 9), b"123456789");
    assert_eq!(
        call(CRC32_LE, [0, DROM, 9], &mut memory, &mut io),
        Ok(returned(Some(0xcbf4_3926), 0))
    );
    assert_eq!(
        call(MEMSET, [DROM, 0, 1], &mut memory, &mut io),
        Err(BoardFault::WriteToReadOnly {
            pc: MEMSET,
            addr: DROM
        })
    );
}

#[test]
fn uart_сохраняет_байты_abi_и_границу_кадра() {
    let mut memory = EspMemory::default();
    let mut io = Io::default();
    // UART QEMU после строки entry: ровно R LF, оба вызова вернули ноль.
    for byte in [u32::from(b'R'), u32::from(b'\n')] {
        assert_eq!(
            call(UART_TX, [byte, 0, 0], &mut memory, &mut io),
            Ok(returned(Some(0), 0))
        );
    }
    assert_eq!(io.tx, b"R\n");
    // QEMU: пустой RX вернул 1 и сохранил буфер 0x37.
    let dst = DATA + 32;
    load(&mut memory, dst, b"7");
    assert_eq!(
        call(UART_RX, [dst, 0, 0], &mut memory, &mut io),
        Ok(returned(Some(1), 0))
    );
    assert_eq!(read(&mut memory, dst, 1), b"7");
    // После подачи Q через последовательный вход QEMU вернула 0 и байт 0x51.
    io.rx = Some(b'Q');
    assert_eq!(
        call(UART_RX, [END_OF_SRAM, 0, 0], &mut memory, &mut io),
        Err(BoardFault::BusError {
            pc: UART_RX,
            addr: END_OF_SRAM,
            write: true
        })
    );
    assert_eq!(io.rx, Some(b'Q'));
    assert_eq!(
        call(UART_RX, [dst, 0, 0], &mut memory, &mut io),
        Ok(returned(Some(0), 0))
    );
    assert_eq!(read(&mut memory, dst, 1), b"Q");
    assert!(io.rx.is_none());

    // Задержка 7 тактов задана стендом готовности FIFO, не частотой UART.
    let wait = NonZeroU64::new(7).unwrap();
    io.tx_busy = Some(wait);
    assert_eq!(
        call(UART_TX, [u32::from(b'!'), 0, 0], &mut memory, &mut io),
        Ok(RomStep::Wait { cycles: wait })
    );
    assert_eq!(io.tx, b"R\n");
    io.tx_busy = None;
    assert_eq!(
        call(UART_TX, [u32::from(b'!'), 0, 0], &mut memory, &mut io),
        Ok(returned(Some(0), 0))
    );
    assert_eq!(io.tx, b"R\n!");
    io.tx_error = true;
    assert_eq!(
        call(UART_TX, [u32::from(b'?'), 0, 0], &mut memory, &mut io),
        Ok(returned(Some(1), 0))
    );
    assert_eq!(io.tx, b"R\n!");
    let fault = BoardFault::Unsupported("режим console стенда".into());
    io.serial_fault = Some(fault.clone());
    assert_eq!(call(UART_TX, [0, 0, 0], &mut memory, &mut io), Err(fault));

    io.uart_busy = Some(wait);
    assert_eq!(
        call(UART_IDLE, [0, 0, 0], &mut memory, &mut io),
        Ok(RomStep::Wait { cycles: wait })
    );
    assert_eq!(io.uart_state, Some((0, UartWait::Idle)));
    io.uart_busy = None;
    assert_eq!(
        call(UART_FLUSH, [1, 0, 0], &mut memory, &mut io),
        Ok(returned(None, 0))
    );
    assert_eq!(io.uart_state, Some((1, UartWait::FifoEmpty)));
    // esp_rom_serial_output.h: USB также может быть console, но модель
    // ожидания поддерживает только два UART из даташита §4.2.1.
    assert!(matches!(
        call(UART_IDLE, [3, 0, 0], &mut memory, &mut io),
        Err(BoardFault::Unsupported(_))
    ));
}

#[test]
fn задержка_берёт_такты_из_rom_и_отклоняет_неопределённые_входы() {
    let mut memory = EspMemory::default();
    let mut io = Io::default();
    assert!(matches!(
        call(DELAY_US, [3, 0, 0], &mut memory, &mut io),
        Err(BoardFault::Unsupported(_))
    ));
    // QEMU: после handoff 80 тактов/мкс, после обновления 40.
    memory.write(GET_TICKS, TICKS, 4, 80).unwrap();
    assert_eq!(
        call(GET_TICKS, [0; 3], &mut memory, &mut io),
        Ok(returned(Some(80), 0))
    );
    assert_eq!(
        call(SET_TICKS, [40, 0, 0], &mut memory, &mut io),
        Ok(returned(None, 0))
    );
    assert_eq!(
        call(GET_TICKS, [0; 3], &mut memory, &mut io),
        Ok(returned(Some(40), 0))
    );
    // 3 мкс × 40 тактов/мкс. В двух пробах PCCR QEMU: 131–132 такта
    // с задержкой и 16–17 с нулём, включая вход, выход и дискретность опроса.
    assert_eq!(
        call(DELAY_US, [3, 0, 0], &mut memory, &mut io),
        Ok(returned(None, 120))
    );
    assert_eq!(
        call(DELAY_US, [0; 3], &mut memory, &mut io),
        Ok(returned(None, 0))
    );
    assert!(matches!(
        call(DELAY_US, [u32::MAX, 0, 0], &mut memory, &mut io),
        Err(BoardFault::Misconfigured(_))
    ));
    assert!(matches!(
        call(SET_TICKS, [0; 3], &mut memory, &mut io),
        Err(BoardFault::Misconfigured(_))
    ));
    assert_eq!(memory.read(GET_TICKS, TICKS, 4), Ok(40));
}

#[test]
fn неизвестный_вход_явно_останавливает_а_printf_сохраняет_полный_abi() {
    let mut memory = EspMemory::default();
    let mut io = Io::default();
    // esp32c3.rom.ld: ets_printf (переменное число аргументов) и
    // ets_get_stack_info (вне поддерживаемого списка).
    assert_eq!(
        call(0x4000_0040, [VECTOR, 1, 2], &mut memory, &mut io),
        Ok(RomStep::Native {
            entry: 0x4000_0040,
            return_pc: RETURN_PC
        })
    );
    for address in [0x4000_0054, 0x4000_062a] {
        assert_eq!(
            call(address, [0; 3], &mut memory, &mut io),
            Err(BoardFault::Unsupported(format!(
                "неизвестный вход ROM ESP32-C3 {address:#010x}"
            )))
        );
    }
}
