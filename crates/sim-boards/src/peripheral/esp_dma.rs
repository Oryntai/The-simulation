//! Дескрипторный GDMA ESP32-C3 (TRM v1.4, гл. 2).
//! Один шаг обмена переносит байт по запросу планировщика или периферии.
//! Задержки AHB, такты FIFO и арбитраж одновременных запросов здесь не вычисляются.

use std::ops::Range;

use crate::{BoardFault, esp_memory::EspMemory, memory::check_data_alignment};

// Даташит §4.1.1.2 и TRM §§2.4.1, 2.4.7. Только окно SRAM1 на шине данных.
const CHANNELS: usize = 3;
const RAM: Range<u32> = 0x3fc8_0000..0x3fce_0000;
const WORD: u32 = size_of::<u32>() as u32;
const DESC_SIZE: u32 = 3 * WORD;
// ESP-IDF esp_hal_dma/include/hal/dma_types.h: dma_descriptor_align4_t.
const SIZE_MASK: u32 = 0xfff;
const LEN_SHIFT: u32 = 12;
const LEN_MASK: u32 = SIZE_MASK << LEN_SHIFT;
const ERR_EOF: u32 = 1 << 28;
const EOF: u32 = 1 << 30;
const OWNER: u32 = 1 << 31;

// Здесь и ниже карта MMIO и поля сверены с ESP-IDF
// soc/esp32c3/register/soc/{reg_base,gdma_reg}.h и TRM §§2.7–2.8.
const BASE: u32 = 0x6003_f000;
const IRQ_STRIDE: u32 = 0x10;
const IRQ_RAW: u32 = 0x00;
const IRQ_ST: u32 = 0x04;
const IRQ_ENA: u32 = 0x08;
const IRQ_CLR: u32 = 0x0c;
const MISC: u32 = 0x44;
const DATE: u32 = 0x48;
const DATE_RESET: u32 = 33_587_792;
const AHB_RESET: u32 = 1 << 0;
const PRI_DISABLE: u32 = 1 << 2;
const REG_CLOCK: u32 = 1 << 3;
const RX_BASE: u32 = 0x70;
const TX_BASE: u32 = 0xd0;
const CHANNEL_STRIDE: u32 = 0xc0;
// Смещения внутри каждого направления от IN_CONF0 или OUT_CONF0.
const CONF0: u32 = 0x00;
const CONF1: u32 = 0x04;
const LINK: u32 = 0x10;
const STATE: u32 = 0x14;
const EOF_DESC: u32 = 0x18;
const OTHER_EOF_DESC: u32 = 0x1c;
const DSCR: u32 = 0x20;
const DSCR_BF0: u32 = 0x24;
const DSCR_BF1: u32 = 0x28;
const PRI: u32 = 0x2c;
const PERI_SEL: u32 = 0x30;
const END: u32 = BASE + TX_BASE + CHANNEL_STRIDE * (CHANNELS as u32 - 1) + PERI_SEL + WORD;
const RESET: u32 = 1 << 0;
const LOOP_TEST: u32 = 1 << 1;
const RX_DESC_BURST: u32 = 1 << 2;
const RX_BURST: u32 = 1 << 3;
const MEM_TRANS: u32 = 1 << 4;
const TX_WRITEBACK: u32 = 1 << 2;
const TX_EOF_MODE: u32 = 1 << 3;
const TX_DESC_BURST: u32 = 1 << 4;
const TX_BURST: u32 = 1 << 5;
const CHECK_OWNER: u32 = 1 << 12;
const FIRST_MASK: u32 = 0x000f_ffff;
const STATE_ADDR_MASK: u32 = 0x0003_ffff;
const RX_AUTO_RETURN: u32 = 1 << 20;
const TX_STOP: u32 = 1 << 20;
const TX_START: u32 = 1 << 21;
const TX_RESTART: u32 = 1 << 22;
const TX_PARK: u32 = 1 << 23;
const PRI_MASK: u32 = 0xf;
const PERI_MASK: u32 = 0x3f;
// Значения GDMA_PERI_{IN,OUT}_SEL из TRM табл. 2.4-1.
const PERIPHERALS: [u8; 6] = [0, 2, 3, 6, 7, 8];
const UHCIO: u8 = 2; // Только этот источник выставляет RX_ERR_EOF (§2.5).

// ESP-IDF esp_hal_dma/esp32c3/include/hal/gdma_ll.h: GDMA_LL_EVENT_*.
const RX_DONE: u32 = 1 << 0;
const RX_EOF: u32 = 1 << 1;
const RX_ERR_EOF: u32 = 1 << 2;
const TX_DONE: u32 = 1 << 3;
const TX_EOF: u32 = 1 << 4;
const RX_DESC_ERR: u32 = 1 << 5;
const TX_DESC_ERR: u32 = 1 << 6;
const RX_DESC_EMPTY: u32 = 1 << 7;
const TX_TOTAL_EOF: u32 = 1 << 8;
const IRQ_MASK: u32 = 0x1fff;

/// Признак конца кадра по TRM §§2.4.1 и 2.4.6.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EspDmaEof {
    None,
    Success,
    Error,
}

/// Байт и соответствующий ему признак конца кадра на интерфейсе периферии.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EspDmaBeat {
    pub byte: u8,
    pub eof: EspDmaEof,
}

/// Три пары RX/TX со своим состоянием дескрипторов и уровнем IRQ.
/// При холодном сбросе тактирование выключено, сброс блока удерживается.
#[derive(Clone)]
pub struct EspDma {
    channels: [Channel; CHANNELS],
    misc: u32,
    date: u32,
    clock: bool,
    reset_held: bool,
}

impl Default for EspDma {
    fn default() -> Self {
        Self {
            channels: std::array::from_fn(|_| Channel::default()),
            misc: 0,
            date: DATE_RESET,
            // ESP-IDF soc/esp32c3/register/soc/system_reg.h: SYSTEM_DMA_CLK_EN/RST.
            clock: false,
            reset_held: true,
        }
    }
}

impl EspDma {
    /// Пары статуса GDMA для матрицы в формате (источник, MMIO INT_ST, маска).
    /// ESP-IDF soc/esp32c3/include/soc/interrupts.h: ETS_DMA_CH0…CH2_INTR_SOURCE.
    pub const IRQ_STATUS: [(u32, u32, u32); CHANNELS] = [
        (44, BASE + IRQ_ST, IRQ_MASK),
        (45, BASE + IRQ_STRIDE + IRQ_ST, IRQ_MASK),
        (46, BASE + 2 * IRQ_STRIDE + IRQ_ST, IRQ_MASK),
    ];

    /// Сбрасывает блок. Уже записанные байты в SRAM остаются в памяти.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// Принимает SYSTEM_DMA_CLK_EN и SYSTEM_DMA_RST. Затвор ставит обмен на паузу,
    /// сброс отменяет его и очищает состояние и маски прерываний.
    pub fn set_bus_control(&mut self, clock: bool, reset: bool) {
        if reset && !self.reset_held {
            self.reset();
        }
        self.clock = clock;
        self.reset_held = reset;
    }

    /// MMIO GDMA, включая маскированные уровни для матрицы IRQ.
    /// За пределами адресного окна возвращает None.
    pub fn access(
        &mut self,
        pc: u32,
        addr: u32,
        size: u8,
        value: Option<u32>,
    ) -> Result<Option<u32>, BoardFault> {
        if !(BASE..END).contains(&addr) {
            return Ok(None);
        }
        let unsupported = || BoardFault::UnsupportedRegister {
            pc,
            addr,
            write: value.is_some(),
            size,
            block: "GDMA ESP32-C3",
        };
        check_data_alignment(pc, addr, size)?;
        if u32::from(size) != WORD {
            return Err(unsupported());
        }
        if value.is_some() && !self.running() {
            return Err(BoardFault::Misconfigured(format!(
                "GDMA: запись в {addr:#010x} при закрытом тактовом затворе или сбросе"
            )));
        }

        let offset = addr - BASE;
        let read_only = |actual| {
            if value.is_some() {
                Err(BoardFault::WriteToReadOnly { pc, addr })
            } else {
                Ok(Some(actual))
            }
        };
        let irq_channel = (offset / IRQ_STRIDE) as usize;
        if let Some(channel) = self.channels.get_mut(irq_channel) {
            let result = match offset % IRQ_STRIDE {
                // RAW = R/WTC/SS, CLR = WT: очистка через соответствующий CLR.
                IRQ_RAW => return read_only(channel.raw),
                IRQ_ST => return read_only(channel.raw & channel.enabled),
                IRQ_ENA => register(value, &mut channel.enabled, IRQ_MASK),
                IRQ_CLR => {
                    let v = value.ok_or_else(&unsupported)?;
                    channel.raw &= !v;
                    0
                }
                _ => return Err(unsupported()),
            };
            return Ok(Some(result));
        }
        match offset {
            MISC => {
                if value.is_some_and(|v| v & AHB_RESET != 0) {
                    return Err(unsupported());
                }
                return Ok(Some(register(
                    value,
                    &mut self.misc,
                    PRI_DISABLE | REG_CLOCK,
                )));
            }
            DATE => return Ok(Some(register(value, &mut self.date, u32::MAX))),
            _ => {}
        }
        let relative = offset.checked_sub(RX_BASE).ok_or_else(&unsupported)?;
        let channel = &mut self.channels[(relative / CHANNEL_STRIDE) as usize];
        let mut reg = relative % CHANNEL_STRIDE;
        let rx = reg < TX_BASE - RX_BASE;
        let link = if rx {
            &mut channel.rx
        } else {
            reg -= TX_BASE - RX_BASE;
            &mut channel.tx
        };
        let result = match reg {
            CONF0 => {
                let mode = if rx {
                    RX_DESC_BURST | RX_BURST | MEM_TRANS
                } else {
                    TX_WRITEBACK | TX_EOF_MODE | TX_DESC_BURST | TX_BURST
                };
                if value.is_some_and(|v| v & RESET != 0) {
                    link.reset_cursor();
                }
                register(value, &mut link.conf, RESET | LOOP_TEST | mode)
            }
            CONF1 => register(value, &mut link.owner_check, CHECK_OWNER),
            LINK => match value {
                Some(v) => {
                    link.command(v, rx)?;
                    0
                }
                None => link.control | if link.active() { 0 } else { park(rx) },
            },
            STATE => {
                let addr = if link.lookahead == 0 {
                    link.current
                } else {
                    link.lookahead
                };
                return read_only(addr & STATE_ADDR_MASK);
            }
            EOF_DESC => return read_only(link.eof_desc),
            OTHER_EOF_DESC => return read_only(link.other_eof_desc),
            DSCR => return read_only(link.lookahead),
            DSCR_BF0 => return read_only(link.current),
            DSCR_BF1 => return read_only(link.previous),
            PRI => register(value, &mut link.priority, PRI_MASK),
            PERI_SEL => register(value, &mut link.peripheral, PERI_MASK),
            // Пополнение FIFO, его заполнение и внутренний сброс AHB требуют
            // модели тактов и шины. Не выдаём состояние вымышленной очереди.
            _ => return Err(unsupported()),
        };
        Ok(Some(result))
    }

    /// Пропускает до budget байтов через пару TX→RX того же номера (§2.4.3).
    /// Бюджет задаётся вызывающим. При ошибке уже завершённая часть сохраняется.
    pub fn step_memory(
        &mut self,
        memory: &mut EspMemory,
        channel: usize,
        budget: usize,
    ) -> Result<usize, BoardFault> {
        let running = self.running();
        let pair = self.channels.get_mut(channel).ok_or_else(|| {
            BoardFault::Misconfigured(format!("GDMA: нет пары с номером {channel}"))
        })?;
        if !running || pair.rx.conf & MEM_TRANS == 0 {
            return Ok(0);
        }
        for moved in 0..budget {
            let Some(tx) = pair.tx.prepare(memory, false, &mut pair.raw)? else {
                return Ok(moved);
            };
            let Some(rx) = pair.prepare_rx(memory)? else {
                return Ok(moved);
            };
            let beat = pair.read_tx(memory, tx)?;
            pair.write_rx(memory, rx, beat)?;
            pair.finish_tx(memory, tx)?;
        }
        Ok(budget)
    }

    /// Периферия забирает один байт из выбранного TX. None означает ожидание.
    pub fn transmit(
        &mut self,
        memory: &mut EspMemory,
        peripheral: u8,
    ) -> Result<Option<EspDmaBeat>, BoardFault> {
        check_peripheral(peripheral)?;
        if !self.running() {
            return Ok(None);
        }
        let Some(pair) = self.peripheral_channel(peripheral, false)? else {
            return Ok(None);
        };
        let Some(tx) = pair.tx.prepare(memory, false, &mut pair.raw)? else {
            return Ok(None);
        };
        let beat = pair.read_tx(memory, tx)?;
        pair.finish_tx(memory, tx)?;
        Ok(Some(beat))
    }

    /// Периферия предлагает RX один байт. false означает отсутствие готового канала.
    pub fn receive(
        &mut self,
        memory: &mut EspMemory,
        peripheral: u8,
        beat: EspDmaBeat,
    ) -> Result<bool, BoardFault> {
        check_peripheral(peripheral)?;
        if beat.eof == EspDmaEof::Error && peripheral != UHCIO {
            return Err(BoardFault::Misconfigured(format!(
                "GDMA: ERR_EOF допустим только для UHCIO, получен от периферии {peripheral}"
            )));
        }
        if !self.running() {
            return Ok(false);
        }
        let Some(pair) = self.peripheral_channel(peripheral, true)? else {
            return Ok(false);
        };
        if beat.eof == EspDmaEof::Error && pair.rx.control & RX_AUTO_RETURN != 0 {
            return Err(BoardFault::Unsupported(
                "GDMA: возврат к текущему дескриптору при ошибке UHCIO".into(),
            ));
        }
        let Some(rx) = pair.prepare_rx(memory)? else {
            return Ok(false);
        };
        pair.write_rx(memory, rx, beat)?;
        Ok(true)
    }

    fn running(&self) -> bool {
        self.clock && !self.reset_held
    }

    fn peripheral_channel(
        &mut self,
        peripheral: u8,
        rx: bool,
    ) -> Result<Option<&mut Channel>, BoardFault> {
        let mut pairs = self.channels.iter_mut().filter(|pair| {
            let direction = if rx { &pair.rx } else { &pair.tx };
            pair.rx.conf & MEM_TRANS == 0 && direction.peripheral == u32::from(peripheral)
        });
        let selected = pairs.next();
        if pairs.next().is_some() {
            return Err(BoardFault::Misconfigured(format!(
                "GDMA: периферия {peripheral} подключена к нескольким каналам одного направления"
            )));
        }
        Ok(selected)
    }
}

fn register(value: Option<u32>, dst: &mut u32, mask: u32) -> u32 {
    if let Some(v) = value {
        *dst = v & mask;
        0
    } else {
        *dst
    }
}

fn check_peripheral(peripheral: u8) -> Result<(), BoardFault> {
    if PERIPHERALS.contains(&peripheral) {
        Ok(())
    } else {
        Err(BoardFault::Misconfigured(format!(
            "GDMA: номер периферии {peripheral} вне табл. 2.4-1 TRM"
        )))
    }
}

#[derive(Clone, Default)]
struct Channel {
    rx: Link,
    tx: Link,
    raw: u32,
    enabled: u32,
}

impl Channel {
    fn prepare_rx(&mut self, memory: &mut EspMemory) -> Result<Option<Descriptor>, BoardFault> {
        let desc = self.rx.prepare(memory, true, &mut self.raw)?;
        if desc.is_none() && self.rx.exhausted {
            self.raw |= RX_DESC_EMPTY;
            return Err(fault(
                self.rx.current,
                "список RX закончился до конца данных",
            ));
        }
        Ok(desc)
    }

    fn read_tx(
        &mut self,
        memory: &mut EspMemory,
        tx: Descriptor,
    ) -> Result<EspDmaBeat, BoardFault> {
        let addr = tx.buffer + tx.moved;
        let byte = memory.read(tx.addr, addr, 1).map_err(|error| {
            self.tx.fail(false, &mut self.raw);
            fault(tx.addr, format_args!("чтение буфера {addr:#010x}: {error}"))
        })?;
        Ok(EspDmaBeat {
            byte: byte as u8,
            eof: if tx.flags & EOF != 0 && tx.moved + 1 == tx.len() {
                EspDmaEof::Success
            } else {
                EspDmaEof::None
            },
        })
    }

    fn finish_tx(&mut self, memory: &mut EspMemory, mut tx: Descriptor) -> Result<(), BoardFault> {
        tx.moved += 1;
        if tx.moved < tx.len() {
            self.tx.cursor = Cursor::Transfer(tx);
            return Ok(());
        }
        if self.tx.conf & (TX_WRITEBACK | LOOP_TEST) == TX_WRITEBACK {
            self.tx
                .write_flags(memory, tx.addr, tx.flags & !OWNER, false, &mut self.raw)?;
        }
        self.raw |= TX_DONE;
        if tx.flags & EOF != 0 {
            self.raw |= TX_EOF;
            self.tx.eof_desc = tx.addr;
            self.tx.other_eof_desc = self.tx.previous;
        }
        // TRM §2.5: TOTAL_EOF относится к завершению списка, EOF — к кадру.
        if tx.next == 0 {
            self.raw |= TX_TOTAL_EOF;
        }
        self.tx.complete(tx);
        Ok(())
    }

    fn write_rx(
        &mut self,
        memory: &mut EspMemory,
        mut rx: Descriptor,
        beat: EspDmaBeat,
    ) -> Result<(), BoardFault> {
        let addr = rx.buffer + rx.moved;
        memory
            .write(rx.addr, addr, 1, u32::from(beat.byte))
            .map_err(|error| {
                self.rx.fail(true, &mut self.raw);
                fault(rx.addr, format_args!("запись буфера {addr:#010x}: {error}"))
            })?;
        rx.moved += 1;
        if rx.moved < rx.size() && beat.eof == EspDmaEof::None {
            self.rx.cursor = Cursor::Transfer(rx);
            return Ok(());
        }
        let mut flags = rx.flags & !(EOF | ERR_EOF | LEN_MASK);
        if self.rx.conf & LOOP_TEST == 0 {
            flags &= !OWNER;
        }
        flags |= (rx.moved << LEN_SHIFT)
            | match beat.eof {
                EspDmaEof::None => 0,
                EspDmaEof::Success => EOF,
                EspDmaEof::Error => ERR_EOF,
            };
        self.rx
            .write_flags(memory, rx.addr, flags, true, &mut self.raw)?;
        self.raw |= RX_DONE;
        match beat.eof {
            EspDmaEof::None => {}
            EspDmaEof::Success => {
                self.raw |= RX_EOF;
                self.rx.eof_desc = rx.addr;
            }
            EspDmaEof::Error => {
                self.raw |= RX_ERR_EOF;
                self.rx.other_eof_desc = rx.addr;
            }
        }
        self.rx.complete(rx);
        Ok(())
    }
}

#[derive(Clone)]
struct Link {
    cursor: Cursor,
    control: u32,
    conf: u32,
    owner_check: u32,
    peripheral: u32,
    priority: u32,
    // DSCR, DSCR_BF0 и DSCR_BF1 согласно описаниям регистров в TRM §2.8.
    lookahead: u32,
    current: u32,
    previous: u32,
    eof_desc: u32,
    other_eof_desc: u32,
    exhausted: bool,
    halted: bool,
}

impl Default for Link {
    fn default() -> Self {
        Self {
            cursor: Cursor::Parked,
            control: 0,
            conf: 0,
            owner_check: 0,
            peripheral: PERI_MASK, // GDMA_PERI_*_SEL: сброс в 63.
            priority: 0,
            lookahead: 0,
            current: 0,
            previous: 0,
            eof_desc: 0,
            other_eof_desc: 0,
            exhausted: false,
            halted: false,
        }
    }
}

fn park(rx: bool) -> u32 {
    TX_PARK << u32::from(rx)
}

impl Link {
    fn active(&self) -> bool {
        !matches!(self.cursor, Cursor::Parked)
    }

    fn reset_cursor(&mut self) {
        self.cursor = Cursor::Parked;
        self.lookahead = 0;
        self.current = 0;
        self.previous = 0;
        self.eof_desc = 0;
        self.other_eof_desc = 0;
        self.exhausted = false;
        self.halted = false;
    }

    fn command(&mut self, value: u32, rx: bool) -> Result<(), BoardFault> {
        // gdma_reg.h: RX-команды START/STOP/RESTART сдвинуты на один бит от TX.
        let commands = (value >> u32::from(rx)) & (TX_START | TX_STOP | TX_RESTART);
        let first = (RAM.start & !FIRST_MASK) | (value & FIRST_MASK);
        if commands.count_ones() > 1 {
            return Err(fault(first, "одновременно заданы несколько команд LINK"));
        }
        if commands & (TX_START | TX_RESTART) != 0 && (self.halted || self.conf & RESET != 0) {
            return Err(fault(
                first,
                "канал удерживается в сбросе или требует сброса после ошибки",
            ));
        }
        if commands == TX_START && self.active() {
            return Err(fault(first, "START на активном канале"));
        }
        if commands == TX_RESTART && !self.active() && self.current == 0 {
            return Err(fault(first, "RESTART без загруженного дескриптора"));
        }
        self.control = value & (FIRST_MASK | if rx { RX_AUTO_RETURN } else { 0 });
        match commands {
            TX_START => {
                self.exhausted = false;
                self.cursor = Cursor::Fetch(first);
            }
            TX_STOP => {
                self.exhausted = false;
                self.cursor = Cursor::Parked;
            }
            TX_RESTART => match &mut self.cursor {
                Cursor::Transfer(desc) => desc.reload_next = true,
                Cursor::Parked => self.cursor = Cursor::Follow(self.current),
                Cursor::Fetch(_) | Cursor::Follow(_) => {}
            },
            _ => {}
        }
        Ok(())
    }

    fn prepare(
        &mut self,
        memory: &mut EspMemory,
        rx: bool,
        raw: &mut u32,
    ) -> Result<Option<Descriptor>, BoardFault> {
        let addr = match self.cursor {
            Cursor::Transfer(mut desc) if desc.reload_next => {
                desc.next = self.read_next(memory, desc.addr, rx, raw)?;
                desc.reload_next = false;
                self.lookahead = desc.next;
                self.cursor = Cursor::Transfer(desc);
                return Ok(Some(desc));
            }
            Cursor::Transfer(desc) => return Ok(Some(desc)),
            Cursor::Fetch(addr) => addr,
            Cursor::Follow(addr) => self.read_next(memory, addr, rx, raw)?,
            Cursor::Parked => return Ok(None),
        };
        if addr == 0 {
            self.cursor = Cursor::Parked;
            return Ok(None);
        }
        let result = Descriptor::load(memory, addr, rx, self.conf, self.owner_check != 0);
        match result {
            Ok(desc) => {
                self.previous = self.current;
                self.current = addr;
                self.lookahead = desc.next;
                self.cursor = Cursor::Transfer(desc);
                self.exhausted = false;
                Ok(Some(desc))
            }
            Err(error) => {
                self.fail(rx, raw);
                Err(error)
            }
        }
    }

    fn write_flags(
        &mut self,
        memory: &mut EspMemory,
        addr: u32,
        flags: u32,
        rx: bool,
        raw: &mut u32,
    ) -> Result<(), BoardFault> {
        memory
            .write(addr, addr, WORD as u8, flags)
            .map_err(|error| {
                self.fail(rx, raw);
                fault(addr, error)
            })
    }

    fn read_next(
        &mut self,
        memory: &mut EspMemory,
        addr: u32,
        rx: bool,
        raw: &mut u32,
    ) -> Result<u32, BoardFault> {
        read_next(memory, addr).inspect_err(|_| self.fail(rx, raw))
    }

    fn fail(&mut self, rx: bool, raw: &mut u32) {
        *raw |= if rx { RX_DESC_ERR } else { TX_DESC_ERR };
        self.cursor = Cursor::Parked;
        self.exhausted = false;
        self.halted = true;
    }

    fn complete(&mut self, desc: Descriptor) {
        self.exhausted = desc.next == 0;
        self.cursor = if self.exhausted {
            Cursor::Parked
        } else {
            Cursor::Fetch(desc.next)
        };
    }
}

#[derive(Clone, Copy)]
enum Cursor {
    Parked,
    Fetch(u32),
    Transfer(Descriptor),
    Follow(u32),
}

#[derive(Clone, Copy)]
struct Descriptor {
    addr: u32,
    flags: u32,
    buffer: u32,
    next: u32,
    moved: u32,
    reload_next: bool,
}

impl Descriptor {
    fn load(
        memory: &mut EspMemory,
        addr: u32,
        rx: bool,
        conf: u32,
        check_owner: bool,
    ) -> Result<Self, BoardFault> {
        if !descriptor_in_ram(addr) {
            return Err(fault(
                addr,
                "адрес не выровнен или дескриптор выходит за SRAM1",
            ));
        }
        let flags = read_word(memory, addr, addr)?;
        if check_owner && flags & OWNER == 0 {
            return Err(fault(addr, "буфер принадлежит CPU"));
        }
        let desc = Self {
            addr,
            flags,
            buffer: read_word(memory, addr, addr + WORD)?,
            next: read_next(memory, addr)?,
            moved: 0,
            reload_next: false,
        };
        if desc.size() == 0 || (!rx && !(1..=desc.size()).contains(&desc.len())) {
            return Err(fault(addr, "недопустимые Length/Size дескриптора"));
        }
        if !in_ram(desc.buffer, desc.size()) {
            return Err(fault(
                addr,
                format_args!(
                    "буфер {:#010x} длиной {} байт вне SRAM1",
                    desc.buffer,
                    desc.size()
                ),
            ));
        }
        if rx
            && conf & RX_BURST != 0
            && (!desc.buffer.is_multiple_of(WORD) || !desc.size().is_multiple_of(WORD))
        {
            return Err(fault(
                addr,
                "для RX burst адрес буфера и Size должны быть кратны слову",
            ));
        }
        Ok(desc)
    }

    fn size(self) -> u32 {
        self.flags & SIZE_MASK
    }

    fn len(self) -> u32 {
        (self.flags & LEN_MASK) >> LEN_SHIFT
    }
}

fn in_ram(addr: u32, len: u32) -> bool {
    RAM.contains(&addr) && len <= RAM.end - addr
}

fn descriptor_in_ram(addr: u32) -> bool {
    addr.is_multiple_of(WORD) && in_ram(addr, DESC_SIZE)
}

fn read_next(memory: &mut EspMemory, desc: u32) -> Result<u32, BoardFault> {
    let next = read_word(memory, desc, desc + 2 * WORD)?;
    // TRM §§2.4.5, 2.8: DW2 вне SRAM1 или без выравнивания даёт DSCR_ERR,
    // включая случай, когда этот же дескриптор запрашивает EOF.
    if next != 0 && !descriptor_in_ram(next) {
        return Err(fault(
            desc,
            format_args!("следующий дескриптор {next:#010x} не выровнен или выходит за SRAM1"),
        ));
    }
    Ok(next)
}

fn read_word(memory: &mut EspMemory, desc: u32, addr: u32) -> Result<u32, BoardFault> {
    memory
        .read(desc, addr, WORD as u8)
        .map_err(|error| fault(desc, error))
}

fn fault(desc: u32, reason: impl std::fmt::Display) -> BoardFault {
    BoardFault::Misconfigured(format!("GDMA, дескриптор {desc:#010x}: {reason}"))
}
