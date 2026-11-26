//! Холодный старт ESP32-C3 и ABI вызовов ROM.

use std::{mem::size_of, ops::Range, sync::Arc};

use crate::{
    BoardFault,
    esp_image::{SegmentKind, parse_flash},
    esp_memory::EspMemory,
};

// TRM v1.4 §6.1.3 и §3.3.2. Вектор в начале IBUS также виден в
// QEMU Espressif по `info registers` перед первой инструкцией.
pub(super) const RESET_VECTOR: u32 = 0x4000_0000;
const ROM: Range<u32> = RESET_VECTOR..0x4006_0000;

pub(super) fn contains(pc: u32) -> bool {
    ROM.contains(&pc)
}

/// Разрешённые входы после передачи управления из ROM. Тело функции
/// исполняется из дампа со штатным ABI RV32, включая вызовы через таблицу ROM.
pub(super) fn check_guest_entry(target: u32) -> Result<(), BoardFault> {
    if !calls::known(target) {
        return Err(calls::unknown_entry(target));
    }
    Ok(())
}

// ESP-IDF bootloader/subproject/main/ld/esp32c3/bootloader.memory.ld.in:
// iram_dram_offset, __stack_sentry и карта статических данных ROM.
// esp_system/ld/esp32c3/memory.ld.in: SRAM_DRAM_ORG при занятом ICache SRAM0.
const IRAM_DRAM_OFFSET: u32 = 0x0070_0000;
const BOOT_SRAM: Range<u32> = 0x3fc8_0000..0x3fcd_c710;

// TRM §5.8 GPIO_STRAP_REG и §7.2. Порядок уровней на входе API следует номерам GPIO.
const STRAP_PINS: [usize; 3] = [2, 8, 9];
const GPIO_STRAP_REG: u32 = 0x6000_4038;
const STRAP_SPI_BOOT: u32 = 1 << 3;
const STRAP_JOINT_DOWNLOAD: u32 = 1 << 2;

// ESP-IDF soc/esp32c3/register/soc/{reg_base,efuse_reg}.h:
// EFUSE_RD_REPEAT_DATA1_REG/SPI_BOOT_CRYPT_CNT и DATA2_REG/SECURE_BOOT_EN.
const EFUSE_CRYPT: u32 = 0x6000_8834;
const EFUSE_CRYPT_CNT: u32 = 0b111 << 18;
const EFUSE_SECURE: u32 = 0x6000_8838;
const EFUSE_SECURE_BOOT_EN: u32 = 1 << 20;
const WORD: u32 = size_of::<u32>() as u32;

struct Segment {
    sram: Range<u32>,
    flash_start: usize,
}

/// Неизменяемые входы, общие для повторных сбросов и снимков.
pub(super) struct BootImage {
    flash: Arc<[u8]>,
    reset_memory: EspMemory,
    straps: [bool; 3],
    entry: u32,
    segments: Vec<Segment>,
}

impl BootImage {
    pub(super) fn new(
        flash: &[u8],
        table_offset: u32,
        rom: &[u8],
        efuse: &[u32],
        straps: [bool; 3],
    ) -> Result<Self, BoardFault> {
        let mut reset_memory = EspMemory::default();
        reset_memory.install_rom(rom)?;
        reset_memory.install_efuse_readback(efuse)?;

        // TRM §23.3: нечётное число прожжённых битов включает расшифровку flash.
        if (reset_memory.read(RESET_VECTOR, EFUSE_CRYPT, WORD as u8)? & EFUSE_CRYPT_CNT)
            .count_ones()
            % 2
            != 0
            || reset_memory.read(RESET_VECTOR, EFUSE_SECURE, WORD as u8)? & EFUSE_SECURE_BOOT_EN
                != 0
        {
            return Err(BoardFault::Unsupported(
                "первичная загрузка с Secure Boot или шифрованием flash".into(),
            ));
        }

        let parsed = parse_flash(flash, table_offset)?;
        let loader = parsed.bootloader();
        let entry = loader.header.entry_addr;
        let mut segments: Vec<Segment> = Vec::new();
        // TRM §7.2 Normal Flash Boot: ROM загружает вторую ступень в SRAM.
        // DROM/IROM и загрузка приложения принадлежат следующей ступени.
        for seg in &loader.segments {
            let addr = match seg.kind {
                SegmentKind::Padding => continue,
                SegmentKind::Dram => seg.load_addr,
                SegmentKind::Iram => seg.load_addr - IRAM_DRAM_OFFSET,
                _ => {
                    return Err(BoardFault::Unsupported(format!(
                        "тип сегмента второй ступени по адресу {:#010x} вне поддержанной SRAM",
                        seg.load_addr
                    )));
                }
            };
            // Тела сегментов кратны слову по esp_image_format.c, verify_segment_header.
            let len = seg.data.len() as u32;
            if len == 0 {
                continue;
            }
            let end = addr + len; // Границы сегментов и отсутствие переполнения проверил parse_flash.
            if !addr.is_multiple_of(WORD) || !BOOT_SRAM.contains(&addr) || end > BOOT_SRAM.end {
                return Err(BoardFault::BadImage(format!(
                    "сегмент второй ступени {:#010x}..{:#010x} вне SRAM загрузчика или не выровнен",
                    seg.load_addr,
                    seg.load_addr + len
                )));
            }
            if segments
                .iter()
                .any(|s| addr < s.sram.end && s.sram.start < end)
            {
                return Err(BoardFault::BadImage(format!(
                    "сегмент второй ступени по адресу {:#010x} пересекает другой сегмент через SRAM",
                    seg.load_addr
                )));
            }
            segments.push(Segment {
                sram: addr..end,
                flash_start: seg.flash_offset as usize,
            });
        }
        Ok(Self {
            flash: flash.into(),
            reset_memory,
            straps,
            entry,
            segments,
        })
    }

    pub(super) fn cold_memory(&self, pins: &[Option<bool>]) -> EspMemory {
        let [gpio2, gpio8, gpio9] =
            std::array::from_fn(|i| pins[STRAP_PINS[i]].unwrap_or(self.straps[i]));
        let mut memory = self.reset_memory.clone();
        memory.latch_straps(gpio2, gpio8, gpio9);
        memory
    }

    pub(super) fn check_mode(&self, memory: &mut EspMemory) -> Result<(), BoardFault> {
        let straps = memory.read(RESET_VECTOR, GPIO_STRAP_REG, WORD as u8)?;
        // Даташит §3.1 и TRM табл. 7.2-1: GPIO9=1 выбирает SPI Boot.
        if straps & STRAP_SPI_BOOT != 0 {
            return Ok(());
        }
        let mode = if straps & STRAP_JOINT_DOWNLOAD != 0 {
            "Joint Download Boot"
        } else {
            "SPI Download Boot или недопустимая комбинация strap (TRM §7.2)"
        };
        Err(BoardFault::Unsupported(format!(
            "режим ROM ESP32-C3: {mode}"
        )))
    }

    pub(super) fn check_transfer(
        &self,
        in_rom: &mut bool,
        pc: u32,
        memory: &mut EspMemory,
    ) -> Result<(), BoardFault> {
        if !*in_rom || ROM.contains(&pc) {
            return Ok(());
        }
        if pc != self.entry {
            return Err(BoardFault::Unsupported(format!(
                "ROM передала управление на {pc:#010x}, ожидался вход второй ступени {:#010x}",
                self.entry
            )));
        }
        for seg in &self.segments {
            for (i, addr) in seg.sram.clone().step_by(WORD as usize).enumerate() {
                let start = seg.flash_start + i * WORD as usize;
                let expected = u32::from_le_bytes(
                    self.flash[start..start + WORD as usize]
                        .try_into()
                        .expect("слово проверенного сегмента образа"),
                );
                if memory.read(pc, addr, WORD as u8)? != expected {
                    return Err(BoardFault::BadImage(format!(
                        "ROM загрузила неверное слово второй ступени по адресу {addr:#010x}"
                    )));
                }
            }
        }
        *in_rom = false;
        Ok(())
    }
}

/// Функциональный эталон ABI для тестов ROM. CPU исполняет исходный дамп.
#[allow(
    dead_code,
    reason = "функциональный эталон проверяется в tests/esp_rom.rs"
)]
pub(super) mod calls {
    use std::{mem::size_of, num::NonZeroU64, ops::Range};

    use super::ROM;
    use crate::{BoardFault, esp_memory::EspMemory};

    // TRM §3.3.1–2, табл. 3.3-1: SRAM1 с двумя алиасами, RTC FAST, ROM1 DBUS.
    // SRAM0 не используется: доступ к банку зависит от состояния ICache (§14.4.2).
    const DRAM: Range<u32> = 0x3fc8_0000..0x3fce_0000;
    const IRAM: Range<u32> = 0x4038_0000..0x403e_0000;
    const RTC_FAST: Range<u32> = 0x5000_0000..0x5000_2000;
    const DROM1: Range<u32> = 0x3ff0_0000..0x3ff2_0000;
    // ESP-IDF soc/esp32c3/include/soc/ext_mem_defs.h, SOC_DRAM0_CACHE_ADDRESS_LOW/HIGH.
    const DROM_FLASH: Range<u32> = 0x3c00_0000..0x3c80_0000;
    const WORD_BYTES: u32 = size_of::<u32>() as u32;
    const BYTE_BITS: u32 = u8::BITS;

    // ESP-IDF tools/tools.json, инструмент esp-rom-elfs:
    // s_ticks_per_us из esp32c3_rev3_rom.elf, чтение/запись в
    // ets_get_cpu_frequency/ets_update_cpu_frequency. Контракт — esp_rom_sys.h.
    const ROM_TICKS_PER_US: u32 = 0x3fcd_f64c;

    // ESP-IDF esp_rom_spiflash.h: enum esp_rom_spiflash_result_t.
    const FLASH_OK: u32 = 0;
    const FLASH_ERROR: u32 = 1;
    const FLASH_TIMEOUT: u32 = 2;
    // ESP-IDF esp_rom_serial_output.h: коды возврата TX/RX.
    const UART_OK: u32 = 0;
    const UART_ERROR: u32 = 1;

    #[derive(Clone, Copy)]
    pub struct RomCall {
        pub entry: u32,
        pub ra: u32,
        pub args: [u32; 3],
    }

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub enum RomStep {
        /// Продвинуть периферию и счётчик CPU на hold_cycles до исполнения
        /// по pc. Если ABI определяет результат, value задаёт a0.
        Return {
            pc: u32,
            value: Option<u32>,
            hold_cycles: u64,
        },
        /// Оставить PC и аргументы на входе, продвинуть время и проверить заново.
        Wait { cycles: NonZeroU64 },
        /// Исполнить подлинный код из установленного дампа со всеми регистрами,
        /// стеком и ловушками, до возврата в вызывающий код.
        Native { entry: u32, return_pc: u32 },
    }

    #[derive(Clone, Copy)]
    pub enum FlashRead<'a> {
        Bytes(&'a [u8]),
        Error,
        Timeout,
    }

    pub enum UartTx {
        Sent,
        Busy(NonZeroU64),
        Error,
    }

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub enum UartWait {
        FifoEmpty,
        Idle,
    }

    pub trait RomIo {
        /// Физическое чтение без кэша. Отсутствие байтов снимка и
        /// неподдержанный режим передаются как BoardFault.
        fn read_flash(&mut self, offset: u32, len: usize) -> Result<FlashRead<'_>, BoardFault>;

        /// Чтение окна DROM с проверкой текущего кэша и MMU.
        fn read_cached_byte(&mut self, pc: u32, addr: u32) -> Result<u8, BoardFault>;

        /// Принять байт на настроенные ROM каналы вывода. Busy не меняет FIFO
        /// и даёт число тактов CPU до следующей проверки готовности.
        fn uart_send_console(&mut self, byte: u8) -> Result<UartTx, BoardFault>;
        /// Принять байт из настроенного ROM канала ввода, если он уже доступен.
        fn uart_receive_console(&mut self) -> Result<Option<u8>, BoardFault>;
        /// None означает подтверждённое состояние UART; Some — такты
        /// CPU до следующей проверки. Неизвестное время — ошибка модели.
        fn uart_wait(
            &mut self,
            serial: u8,
            condition: UartWait,
        ) -> Result<Option<NonZeroU64>, BoardFault>;
    }

    #[derive(Clone, Copy)]
    enum Function {
        FlashRead,
        Crc {
            polynomial: u32,
            width: u32,
            reflected: bool,
        },
        Copy {
            overlap: bool,
        },
        Fill,
        Compare,
        Transmit,
        Receive,
        WaitUart(UartWait),
        Delay,
        GetTicks,
        SetTicks,
        Native,
    }

    impl Function {
        fn decode(entry: u32) -> Option<Self> {
            use Function::*;
            // ESP-IDF esp_rom/esp32c3/ld/esp32c3.rom{,.api,.libc,
            // .libc-suboptimal_for_misaligned_mem}.ld.
            // Полиномы и разрядность из esp_rom/include/esp_rom_crc.h.
            Some(match entry {
                0x4000_0130 => FlashRead, // esp_rom_spiflash_read
                0x4000_0628 => Crc {
                    polynomial: 0x04c1_1db7,
                    width: 32,
                    reflected: true,
                },
                0x4000_062c => Crc {
                    polynomial: 0x04c1_1db7,
                    width: 32,
                    reflected: false,
                },
                0x4000_0630 => Crc {
                    polynomial: 0x1021,
                    width: 16,
                    reflected: true,
                },
                0x4000_0634 => Crc {
                    polynomial: 0x1021,
                    width: 16,
                    reflected: false,
                },
                0x4000_0638 => Crc {
                    polynomial: 0x07,
                    width: 8,
                    reflected: true,
                },
                0x4000_063c => Crc {
                    polynomial: 0x07,
                    width: 8,
                    reflected: false,
                },
                0x4000_0354 => Fill,                          // memset
                0x4000_0358 => Copy { overlap: false },       // memcpy
                0x4000_035c => Copy { overlap: true },        // memmove
                0x4000_0360 => Compare,                       // memcmp
                0x4000_0068 => Transmit,                      // uart_tx_one_char
                0x4000_0070 => Receive,                       // uart_rx_one_char
                0x4000_0080 => WaitUart(UartWait::FifoEmpty), // uart_tx_flush
                0x4000_0084 => WaitUart(UartWait::Idle),      // uart_tx_wait_idle
                0x4000_0050 => Delay,                         // ets_delay_us
                0x4000_0584 => GetTicks,                      // ets_get_cpu_frequency
                0x4000_0588 => SetTicks,                      // ets_update_cpu_frequency
                // Эти функции требуют исходных регистров, стека или MMIO:
                // ets_printf, ets_install_uart_printf, SPI wait_idle и config_param.
                0x4000_0040 | 0x4000_0048 | 0x4000_010c | 0x4000_0134 => Native,
                // esp32c3.rom.ld: reset reason, APB backup lock, cache boot init,
                // Cache_Invalidate_Addr, Cache_{Disable,Enable,Suspend,Resume}_ICache,
                // Freeze и размер IDROM MMU.
                0x4000_0018 | 0x4000_0060 | 0x4000_04c8 | 0x4000_04d4 | 0x4000_051c
                | 0x4000_0520 | 0x4000_0524 | 0x4000_0528 | 0x4000_052c | 0x4000_0530
                | 0x4000_0548 => Native,
                // esp32c3.rom.ld: rom_gpio_matrix_{in,out}, rom_gpio_pad_select_gpio, rom_gpio_pad_set_drv,
                // esprv_intc и intr_matrix_set.
                0x4000_05a0 | 0x4000_05a4 | 0x4000_05c8 | 0x4000_05cc | 0x4000_05e0
                | 0x4000_05e4 | 0x4000_05e8 | 0x4000_05ec | 0x4000_05f0 | 0x4000_05f4 => Native,
                // esp32c3.rom.ld: MD5{Init,Update,Final}, ets_sha_{enable,init,update,finish},
                // ets_efuse_get_{spiconfig,wp_pad}. MMIO функций проходит через шину.
                0x4000_0614 | 0x4000_0618 | 0x4000_061c | 0x4000_0644 | 0x4000_0650
                | 0x4000_065c | 0x4000_0660 | 0x4000_071c | 0x4000_072c => Native,
                // esp32c3.rom.libc.ld: strlen, strnlen, div, qsort.
                // esp32c3.rom.libgcc.ld: __ashldi3, __ashrdi3, __clzsi2, __ffssi2,
                // __lshrdi3, __udivdi3, __umoddi3.
                0x4000_0374 | 0x4000_0404 | 0x4000_0428 | 0x4000_0434 | 0x4000_077c
                | 0x4000_0780 | 0x4000_079c | 0x4000_07d4 | 0x4000_0830 | 0x4000_08ac
                | 0x4000_08bc => Native,
                // ESP-IDF esp32c3.rom.libc.ld и esp_libc/src/locks.c:
                // esp_rom_newlib_init_common_mutexes, инициализация блокировок ROM.
                0x4000_0350 => Native,
                // ESP-IDF esp32c3.rom.ld: esp_coex_rom_version_get
                // возвращает строку версии из ROM. Обмен через RF не моделируется.
                0x4000_18ac => Native,
                // ESP-IDF esp_hal_regi2c/esp32c3/rom.regi2c.ld:6–9.
                // ROM выполняет чтение и запись, в том числе с маской, через MMIO.
                0x4000_1954 | 0x4000_1958 | 0x4000_195c | 0x4000_1960 => Native,
                _ => return None,
            })
        }
    }

    pub(super) fn known(entry: u32) -> bool {
        Function::decode(entry).is_some()
    }

    /// Выполнить функциональный эффект одного вызова по RV32 psABI §2.1.
    /// hold_cycles задаёт обязательное ожидание, накладные такты инструкций
    /// и длительность SPI относятся к модели CPU и периферии.
    pub fn dispatch(
        call: RomCall,
        memory: &mut EspMemory,
        io: &mut impl RomIo,
    ) -> Result<RomStep, BoardFault> {
        let RomCall {
            entry,
            ra,
            args: [a0, a1, a2],
        } = call;
        let function = Function::decode(entry).ok_or_else(|| unknown_entry(entry))?;
        // RV32I JALR, псевдоинструкция RET: нулевой младший бит адреса перехода.
        let pc = ra & !1;
        let returned = |value, hold_cycles| RomStep::Return {
            pc,
            value,
            hold_cycles,
        };
        let wait = |cycles| RomStep::Wait { cycles };
        let value = match function {
            Function::Native => {
                return Ok(RomStep::Native {
                    entry,
                    return_pc: pc,
                });
            }
            Function::FlashRead => Some(flash_read(memory, io, entry, a0, a1, a2)?),
            Function::Crc {
                polynomial,
                width,
                reflected,
            } => {
                let bytes = read_bytes(memory, io, data_span(entry, a1, a2, false)?, entry)?;
                Some(crc(a0, &bytes, polynomial, width, reflected))
            }
            Function::Copy { overlap } => {
                let dst = data_span(entry, a0, a2, true)?;
                let src = data_span(entry, a1, a2, false)?;
                if !overlap && a2 != 0 && src.start < dst.end && dst.start < src.end {
                    return Err(BoardFault::Misconfigured(format!(
                        "memcpy ROM {entry:#010x}: диапазоны {src:?} и {dst:?} перекрываются"
                    )));
                }
                let bytes = read_bytes(memory, io, src, entry)?;
                for (addr, byte) in dst.zip(bytes) {
                    memory.write(entry, addr, 1, u32::from(byte))?;
                }
                Some(a0)
            }
            Function::Fill => {
                for addr in data_span(entry, a0, a2, true)? {
                    // memset использует представление int как unsigned char.
                    memory.write(entry, addr, 1, u32::from(a1 as u8))?;
                }
                Some(a0)
            }
            Function::Compare => {
                let left = data_span(entry, a0, a2, false)?;
                let right = data_span(entry, a1, a2, false)?;
                let mut result = 0;
                for (a, b) in left.zip(right) {
                    // Espressif esp32c3_rev3_rom.elf, memcmp: разность первых
                    // отличающихся unsigned char, со знаком в a0.
                    let diff = i32::from(read_byte(memory, io, entry, a)?)
                        - i32::from(read_byte(memory, io, entry, b)?);
                    if diff != 0 {
                        result = diff;
                        break;
                    }
                }
                Some(result as u32)
            }
            Function::Transmit => match io.uart_send_console(a0 as u8)? {
                UartTx::Sent => Some(UART_OK),
                UartTx::Error => Some(UART_ERROR),
                UartTx::Busy(cycles) => return Ok(wait(cycles)),
            },
            Function::Receive => {
                data_span(entry, a0, 1, true)?;
                if let Some(byte) = io.uart_receive_console()? {
                    memory.write(entry, a0, 1, u32::from(byte))?;
                    Some(UART_OK)
                } else {
                    Some(UART_ERROR)
                }
            }
            Function::WaitUart(condition) => {
                // ESP-IDF esp_rom_uart.h (ESP_ROM_UART_0/1) и TRM §26.2.
                let serial = a0 as u8;
                if !matches!(serial, 0 | 1) {
                    return Err(BoardFault::Unsupported(format!(
                        "ROM {entry:#010x}: ожидание последовательного канала {serial}"
                    )));
                }
                if let Some(cycles) = io.uart_wait(serial, condition)? {
                    return Ok(wait(cycles));
                }
                None
            }
            Function::Delay => {
                let ticks = ticks(memory, entry)?;
                // Espressif esp32c3_rev3_rom.elf, ets_delay_us: 32-битное
                // произведение в цикле опроса PCCR (TRM регистр 1.14).
                let cycles = a0.checked_mul(ticks).ok_or_else(|| {
                    BoardFault::Misconfigured(format!(
                        "ROM {entry:#010x}: {a0} мкс × {ticks} тактов/мкс вне диапазона RV32"
                    ))
                })?;
                return Ok(returned(None, u64::from(cycles)));
            }
            Function::GetTicks => Some(ticks(memory, entry)?),
            Function::SetTicks => {
                require_ticks(entry, a0)?;
                memory.write(entry, ROM_TICKS_PER_US, WORD_BYTES as u8, a0)?;
                None
            }
        };
        Ok(returned(value, 0))
    }

    pub(super) fn unknown_entry(entry: u32) -> BoardFault {
        BoardFault::Unsupported(format!("неизвестный вход ROM ESP32-C3 {entry:#010x}"))
    }

    fn require_ticks(pc: u32, ticks: u32) -> Result<u32, BoardFault> {
        if ticks == 0 {
            return Err(BoardFault::Misconfigured(format!(
                "ROM {pc:#010x}: ноль тактов CPU на микросекунду"
            )));
        }
        Ok(ticks)
    }

    fn ticks(memory: &mut EspMemory, entry: u32) -> Result<u32, BoardFault> {
        require_ticks(
            entry,
            memory.read(entry, ROM_TICKS_PER_US, WORD_BYTES as u8)?,
        )
    }

    fn flash_read(
        memory: &mut EspMemory,
        io: &mut impl RomIo,
        pc: u32,
        offset: u32,
        dest: u32,
        len: u32,
    ) -> Result<u32, BoardFault> {
        // esp_rom_spiflash.h: int32_t len; источник, размер и слова приёмника
        // выровнены на sizeof(uint32_t). Отклоняем запросы вне контракта.
        if len > i32::MAX as u32 || !len.is_multiple_of(WORD_BYTES) {
            return Err(BoardFault::Misconfigured(format!(
                "ROM {pc:#010x}: недопустимая длина чтения flash {len:#x}"
            )));
        }
        for addr in [offset, dest] {
            if !addr.is_multiple_of(WORD_BYTES) {
                return Err(BoardFault::Unaligned {
                    pc,
                    addr,
                    size: WORD_BYTES as u8,
                });
            }
        }
        if offset.checked_add(len).is_none() {
            return Ok(FLASH_ERROR);
        }
        let bytes = match io.read_flash(offset, len as usize)? {
            FlashRead::Error => return Ok(FLASH_ERROR),
            FlashRead::Timeout => return Ok(FLASH_TIMEOUT),
            FlashRead::Bytes(bytes) if bytes.len() == len as usize => bytes,
            FlashRead::Bytes(bytes) => {
                return Err(BoardFault::Misconfigured(format!(
                    "ROM {pc:#010x}: flash вернула {} байт на запрос {len}",
                    bytes.len()
                )));
            }
        };

        let span = if IRAM.contains(&dest) {
            span_in(pc, dest, len, true, &IRAM)?
        } else {
            data_span(pc, dest, len, true)?
        };
        for (addr, word) in span
            .step_by(WORD_BYTES as usize)
            .zip(bytes.chunks_exact(WORD_BYTES as usize))
        {
            let word: [u8; size_of::<u32>()] = word.try_into().expect("слово SPI");
            memory.write(pc, addr, WORD_BYTES as u8, u32::from_le_bytes(word))?;
        }
        Ok(FLASH_OK)
    }

    fn data_span(pc: u32, addr: u32, len: u32, write: bool) -> Result<Range<u32>, BoardFault> {
        if len == 0 {
            return Ok(addr..addr);
        }
        for window in [&DRAM, &RTC_FAST] {
            if window.contains(&addr) {
                return span_in(pc, addr, len, write, window);
            }
        }
        for window in [&DROM1, &DROM_FLASH] {
            if window.contains(&addr) {
                if write {
                    return Err(BoardFault::WriteToReadOnly { pc, addr });
                }
                return span_in(pc, addr, len, write, window);
            }
        }
        if write && ROM.contains(&addr) {
            return Err(BoardFault::WriteToReadOnly { pc, addr });
        }
        // Байтные функции libc и CRC работают по DBUS. Произвольные
        // MMIO и IBUS-указатели сюда не передаются.
        Err(BoardFault::BusError { pc, addr, write })
    }

    fn span_in(
        pc: u32,
        addr: u32,
        len: u32,
        write: bool,
        window: &Range<u32>,
    ) -> Result<Range<u32>, BoardFault> {
        let end = addr
            .checked_add(len)
            .filter(|&end| end <= window.end)
            .ok_or(BoardFault::BusError {
                pc,
                addr: window.end,
                write,
            })?;
        Ok(addr..end)
    }

    fn read_bytes(
        memory: &mut EspMemory,
        io: &mut impl RomIo,
        addresses: Range<u32>,
        pc: u32,
    ) -> Result<Vec<u8>, BoardFault> {
        addresses
            .map(|addr| read_byte(memory, io, pc, addr))
            .collect()
    }

    fn read_byte(
        memory: &mut EspMemory,
        io: &mut impl RomIo,
        pc: u32,
        addr: u32,
    ) -> Result<u8, BoardFault> {
        if DROM_FLASH.contains(&addr) {
            io.read_cached_byte(pc, addr)
        } else {
            memory.read(pc, addr, 1).map(|byte| byte as u8)
        }
    }

    fn crc(seed: u32, bytes: &[u8], polynomial: u32, width: u32, reflected: bool) -> u32 {
        // esp_rom_crc.h: начальное и конечное дополнение, LE отражает полином.
        let mask = u32::MAX >> (u32::BITS - width);
        let high_bit = 1 << (width - 1);
        let mut value = !seed & mask;
        if reflected {
            let polynomial = polynomial.reverse_bits() >> (u32::BITS - width);
            for &byte in bytes {
                value ^= u32::from(byte);
                for _ in 0..BYTE_BITS {
                    value = (value >> 1) ^ if value & 1 != 0 { polynomial } else { 0 };
                }
            }
        } else {
            for &byte in bytes {
                value ^= u32::from(byte) << (width - BYTE_BITS);
                for _ in 0..BYTE_BITS {
                    value = (value << 1) ^ if value & high_bit != 0 { polynomial } else { 0 };
                }
                value &= mask;
            }
        }
        !value & mask
    }
}
