//! ESP32-C3 TRM v1.4, главы 28 (I²C) и 27 (SPI).
//! I²C также сверён с ESP-IDF v6.1.1: i2c_reg.h, i2c_ll.h и i2c_master.c.

use super::{Access, checked_rw, command, invalid, ro, unsupported};
use crate::{
    BitOrder, BoardFault, BoardInput, BoardOutput, I2cRequest, OutputEvent, SpiMode, SpiRequest,
    TransferStatus,
};
use std::collections::VecDeque;

use super::esp_dma::{EspDma, EspDmaBeat, EspDmaEof};
use crate::esp_clock::XTAL_HZ;
use crate::esp_memory::EspMemory;

// TRM §6.2.3 и §27.5: PLL_F80M_CLK.
const PLL80: u32 = 80_000_000;
// TRM табл. 2.4-1: SPI2 — клиент GDMA 0.
const SPI2_DMA: u8 = 0;
// spi_reg.h: DMA_CONF, CMD и USER.
const DMA_TX: u32 = 1 << 28;
const DMA_RX: u32 = 1 << 27;
const DMA_DUPLEX: u32 = DMA_TX | DMA_RX;
const DMA_FIFO_RESET: u32 = 7 << 29;
// spi_ll_master_init: установлены два поля режима сегмента ведомого;
// при выключенном DMA_SLV_SEG_TRANS_EN они не влияют на мастер.
const DMA_SLAVE_CLEAR: u32 = 3 << 19;
const SPI_USR: u32 = 1 << 24;
const SPI_UPDATE: u32 = 1 << 23;

type CpuRate = (u32, u16);

#[derive(Clone, Copy)]
struct SpiTiming {
    cpu: CpuRate,
    legacy: bool,
}

fn source_to_cpu(ticks: u64, source_hz: u64, cpu: CpuRate) -> u64 {
    // Делители регистров ограничивают числитель. Округляем событие один
    // раз к первому такту CPU на/после фронта выбранного источника.
    (u128::from(ticks) * u128::from(cpu.0)).div_ceil(u128::from(source_hz) * u128::from(cpu.1))
        as u64
}

pub(super) const I2C_FIFO: usize = 32; // i2c_ll.h: I2C_LL_FIFO_LEN.
pub(super) const SPI_BYTES: usize = 64; // W0…W15, 32-битные слова.
// i2c_ll.h: I2C_LL_MASTER_EVENT_INTR.
const I2C_END: u32 = 1 << 3;
const I2C_ARBITRATION: u32 = 1 << 5;
const I2C_COMPLETE: u32 = 1 << 7;
const I2C_TIMEOUT: u32 = 1 << 8;
const I2C_NACK: u32 = 1 << 10;
const I2C_IRQ_MASK: u32 = I2C_END | I2C_ARBITRATION | I2C_COMPLETE | I2C_TIMEOUT | I2C_NACK;
const SPI_COMPLETE: u32 = 1 << 12;

// i2c_reg.h: I2C_CTR, I2C_TO, I2C_CLK_CONF и I2C_COMD.
const MASTER: u32 = 1 << 4;
const START: u32 = 1 << 5;
const FSM_RESET: u32 = 1 << 10;
const UPDATE: u32 = 1 << 11;
const TIMEOUT_EN: u32 = 1 << 5;
const CLOCK_ACTIVE: u32 = 1 << 21;
const CMD_DONE: u32 = 1 << 31;
const CMD_BITS: u32 = 0x3fff;
// i2c_ll.h: I2C_LL_CMD_* и i2c_ll_hw_cmd_t.
const RESTART: u32 = 6;
const WRITE: u32 = 1;
const READ: u32 = 3;
const STOP: u32 = 2;
const END: u32 = 4;
const ACK_CHECK: u32 = 1 << 8;
const SEND_NACK: u32 = 1 << 10;
const OPCODE_SHIFT: u32 = 11;

#[derive(Clone)]
struct Reply {
    status: TransferStatus,
    bytes: Vec<u8>,
}

#[derive(Clone, Copy)]
struct I2cLink {
    address: u8,
    read: bool,
    read_ended: bool,
}

struct I2cCommands {
    link: I2cLink,
    write: Vec<u8>,
    read_len: usize,
    repeated_start: bool,
    starts: u64,
    frames: u64,
    terminal: u32,
    count: usize,
    checks_ack: bool,
}

#[derive(Clone)]
struct I2cPending {
    request: Option<I2cRequest>,
    link: I2cLink,
    terminal: u32,
    commands: usize,
    nominal_end: u64,
    response_due: u64,
    timeout_at: Option<u64>,
    checks_ack: bool,
    nack_complete: u64,
    reply: Option<(u64, Reply)>,
}

#[derive(Clone)]
struct I2c {
    low: u32,
    high: u32,
    control: u32,
    clock: u32,
    timeout: u32,
    filter: u32,
    sda: [u32; 2],
    setup: [u32; 4],
    scl_sp: u32,
    clear_until: Option<u64>,
    nack_stop_at: Option<u64>,
    fifo_conf: u32,
    command: [u32; 8],
    tx: VecDeque<u8>,
    rx: VecDeque<u8>,
    raw: u32,
    enable: u32,
    nack: bool,
    link: Option<I2cLink>,
    pending: Option<I2cPending>,
}

impl Default for I2c {
    fn default() -> Self {
        // i2c_reg.h: reset I2C_CTR, CLK_CONF, TO, FILTER_CFG, START/STOP, FIFO_CONF.
        Self {
            low: 0,
            high: 0,
            control: 0x20b,
            clock: CLOCK_ACTIVE,
            timeout: 0x10,
            filter: 0x300,
            sda: [0; 2],
            setup: [8; 4],
            scl_sp: 0,
            clear_until: None,
            nack_stop_at: None,
            fifo_conf: 0x408b,
            command: [0; 8],
            tx: VecDeque::new(),
            rx: VecDeque::new(),
            raw: 0,
            enable: 0,
            nack: false,
            link: None,
            pending: None,
        }
    }
}

impl I2c {
    fn decode(&self) -> Result<I2cCommands, BoardFault> {
        let bad = || invalid("I²C: несогласованные команды START/WRITE/READ/STOP/END и FIFO");
        let mut tx = self.tx.iter().copied();
        let mut link = self.link;
        let mut awaiting_address = false;
        let mut starts = 0;
        let mut frames = 0;
        let mut write = Vec::new();
        let mut read_len = 0;
        for (i, &raw) in self.command.iter().enumerate() {
            let cmd = raw & CMD_BITS;
            let bytes = (cmd & 0xff) as usize; // i2c_ll_hw_cmd_t.byte_num.
            match cmd >> OPCODE_SHIFT {
                RESTART if cmd == RESTART << OPCODE_SHIFT && !awaiting_address => {
                    starts += 1;
                    awaiting_address = true;
                }
                WRITE if cmd >> 8 & 7 == ACK_CHECK >> 8 && bytes != 0 => {
                    for _ in 0..bytes {
                        let byte = tx.next().ok_or_else(bad)?;
                        frames += 1;
                        if awaiting_address {
                            let address = byte >> 1; // TRM §28.4.5: 7-битный адрес и R/W.
                            if !(0x08..=0x77).contains(&address) {
                                return Err(unsupported(
                                    "I²C: зарезервированный или 10-битный адрес",
                                ));
                            }
                            let read = byte & 1 != 0;
                            if starts > 1
                                && link.is_some_and(|l| l.address != address || l.read || !read)
                            {
                                return Err(unsupported(
                                    "I²C: смена адреса или неподдерживаемый RSTART в одном запросе",
                                ));
                            }
                            link = Some(I2cLink {
                                address,
                                read,
                                read_ended: false,
                            });
                            awaiting_address = false;
                        } else if link.is_some_and(|l| !l.read) && (!self.nack || starts != 0) {
                            write.push(byte);
                        } else {
                            return Err(bad());
                        }
                    }
                }
                READ if cmd & 0x300 == 0
                    && bytes != 0
                    && !awaiting_address
                    && (!self.nack || starts != 0) =>
                {
                    let state = link
                        .as_mut()
                        .filter(|l| l.read && !l.read_ended)
                        .ok_or_else(bad)?;
                    state.read_ended = cmd & SEND_NACK != 0;
                    if state.read_ended && bytes != 1 {
                        return Err(bad());
                    }
                    read_len += bytes;
                    frames += bytes as u64;
                }
                terminal @ (STOP | END) if cmd == terminal << OPCODE_SHIFT && !awaiting_address => {
                    let link = link.ok_or_else(bad)?;
                    if tx.next().is_some()
                        || self.rx.len() + read_len > I2C_FIFO
                        || (terminal == STOP
                            && link.read
                            && !link.read_ended
                            && (!self.nack || starts != 0))
                        || (terminal == END && frames == 0)
                    {
                        return Err(bad());
                    }
                    return Ok(I2cCommands {
                        link,
                        write,
                        read_len,
                        starts,
                        frames,
                        terminal,
                        count: i + 1,
                        checks_ack: frames > read_len as u64,
                        repeated_start: starts > 1 || (starts == 1 && self.link.is_some()),
                    });
                }
                _ => return Err(unsupported("I²C: команда или режим проверки ACK")),
            }
        }
        Err(bad())
    }

    fn timing(&self) -> Result<(u64, u64), BoardFault> {
        // i2c_ll_set_source_clk: XTAL при SEL=0. Интервалы считаются в тактах XTAL.
        if self.clock & !0xff != CLOCK_ACTIVE {
            return Err(unsupported("I²C: требуется активный XTAL и целый делитель"));
        }
        let sclk_cycles = u64::from(self.clock & 255) + 1;
        // i2c_ll_master_set_bus_timing: LOW записывается с −1, HIGH и WAIT_HIGH без −1.
        let high = (self.high & 0x1ff) + (self.high >> 9);
        if high == 0 {
            return Err(invalid("I²C: не задан высокий полупериод SCL"));
        }
        let bit_cycles = u64::from(self.low + 1 + high) * sclk_cycles;
        Ok((sclk_cycles, bit_cycles))
    }

    fn start(&mut self, now: u64, id: u64, cpu: CpuRate) -> Result<Option<I2cRequest>, BoardFault> {
        if let Some(pending) = &self.pending {
            // i2c_master.c::s_i2c_send_commands повторно выставляет TRANS_START
            // после запуска восстановительного STOP. В движке исполняется один STOP.
            if pending.request.is_none() {
                return Ok(None);
            }
            return Err(invalid("I²C уже выполняет транзакцию"));
        }
        if self.clear_until.is_some()
            || self.nack_stop_at.is_some()
            || self.control & MASTER == 0
            || self.control & 3 != 3 && self.timeout & TIMEOUT_EN != 0
        {
            return Err(unsupported(
                "I²C: требуется свободный контроллер master/open-drain",
            ));
        }
        let commands = self.decode()?;
        let (sclk_cycles, bit_cycles) = self.timing()?;
        // i2c_ll_master_set_bus_timing записывает четыре интервала с −1.
        let [start_hold, restart_setup, stop_hold, stop_setup] =
            self.setup.map(|value| (u64::from(value) + 1) * sclk_cycles);
        let first_start = if commands.starts == 0 {
            0
        } else if self.link.is_some() {
            restart_setup + start_hold
        } else {
            start_hold
        };
        let extra_starts = commands.starts.saturating_sub(1) * (restart_setup + start_hold);
        let stop_cycles = stop_hold + stop_setup;
        let stop = if commands.terminal == STOP {
            stop_cycles
        } else {
            0
        };
        // TRM §28.4.5, §28.5: 8 бит данных/адреса и один ACK на байт.
        let at = |ticks| now + source_to_cpu(ticks, u64::from(XTAL_HZ), cpu);
        let response_ticks = first_start + 9 * bit_cycles;
        let response_due = at(response_ticks);
        let nominal_end = at(first_start + extra_starts + 9 * commands.frames * bit_cycles + stop);
        let timeout_at = (self.timeout & TIMEOUT_EN != 0).then(|| {
            // TRM §28.8, I2C_TO_REG: 2^TIME_OUT_VALUE тактов I2C_SCLK.
            // В пакетной модели внешний ответ может удерживать SCL у границы первого ACK.
            at(response_ticks + (1_u64 << (self.timeout & 31)) * sclk_cycles)
        });
        let request = (commands.frames != 0).then(|| I2cRequest {
            bus: 0,
            id,
            address: commands.link.address,
            write: commands.write,
            read_len: commands.read_len,
            repeated_start: commands.repeated_start,
            cycles_per_bit: source_to_cpu(bit_cycles, u64::from(XTAL_HZ), cpu),
            address_ack_cycle: response_due,
            complete_cycle: nominal_end,
        });
        for cmd in &mut self.command {
            *cmd &= !CMD_DONE;
        }
        self.tx.clear();
        self.pending = Some(I2cPending {
            request: request.clone(),
            link: commands.link,
            terminal: commands.terminal,
            commands: commands.count,
            nominal_end,
            response_due,
            timeout_at,
            checks_ack: commands.checks_ack,
            nack_complete: at(response_ticks + stop_cycles),
            reply: None,
        });
        Ok(request)
    }

    fn access(
        &mut self,
        at: u32,
        val: Option<u32>,
        now: u64,
        id: u64,
        completed: &mut Vec<OutputEvent>,
        cpu: CpuRate,
    ) -> Result<(Option<u32>, Option<I2cRequest>), BoardFault> {
        // Карта и поля регистров: ESP-IDF v6.1.1 soc/esp32c3/register/soc/i2c_reg.h.
        if val.is_some() && self.pending.is_some() && !matches!(at, 0x04 | 0x24 | 0x28) {
            return Err(invalid("изменение I²C во время транзакции"));
        }
        let result = match (at, val) {
            (0, _) => checked_rw(val, &mut self.low, 0x1ff, "I²C SCL_LOW_PERIOD"),
            (4, Some(value)) => {
                if value & !0xf3f != 0 {
                    return Err(unsupported("I²C CTR: направление или режим"));
                }
                // CONF_UPGATE и TRANS_START — импульсные биты.
                let control = value & !(UPDATE | START | FSM_RESET);
                if self.pending.is_some() && value & FSM_RESET == 0 && control != self.control {
                    return Err(invalid("изменение CTR I²C во время транзакции"));
                }
                if value & FSM_RESET != 0 {
                    if value & START != 0 {
                        return Err(invalid("I²C: FSM_RST и TRANS_START одновременно"));
                    }
                    completed.extend(self.finish(now, true));
                    self.link = None;
                    self.nack = false;
                    self.clear_until = None;
                    self.nack_stop_at = None;
                    self.scl_sp &= !1; // I2C_SCL_RST_SLV_EN.
                }
                let previous = self.control;
                self.control = control;
                if value & START != 0 {
                    let request = self
                        .start(now, id, cpu)
                        .inspect_err(|_| self.control = previous)?;
                    return Ok((Some(0), request));
                }
                Ok(Some(0))
            }
            (4, None) => ro(val, self.control),
            (8, _) => ro(
                val,
                ((self.tx.len() as u32) << 18)
                    | ((self.rx.len() as u32) << 8)
                    | (u32::from(
                        self.pending.is_some() || self.link.is_some() || self.clear_until.is_some(),
                    ) << 4)
                    | u32::from(self.nack),
            ), // I2C_SR: TXFIFO_CNT, RXFIFO_CNT, BUS_BUSY, RESP_REC.
            (0xc, _) => checked_rw(val, &mut self.timeout, 0x3f, "I²C TO"),
            (0x18, Some(value)) if value & !0x73ff == 0 => {
                if value & (1 << 13) != 0 {
                    self.tx.clear();
                } // I2C_TX_FIFO_RST.
                if value & (1 << 12) != 0 {
                    self.rx.clear();
                } // I2C_RX_FIFO_RST.
                self.fifo_conf = value & 0x43ff;
                Ok(Some(0))
            }
            (0x18, None) => ro(val, self.fifo_conf),
            (0x18, _) => Err(unsupported("I²C FIFO_CONF: поддерживается только FIFO")),
            (0x1c, Some(byte)) if byte <= u32::from(u8::MAX) && self.tx.len() < I2C_FIFO => {
                self.tx.push_back(byte as u8);
                Ok(Some(0))
            }
            (0x1c, Some(_)) => Err(invalid("I²C: плохой байт или полный TX FIFO")),
            (0x1c, None) => self
                .rx
                .pop_front()
                .map(|b| Some(u32::from(b)))
                .ok_or_else(|| invalid("I²C: RX FIFO пуст")),
            (0x20, _) => ro(val, self.raw),
            (0x24, _) => command(val, |bits| self.raw &= !bits),
            (0x28, _) => checked_rw(val, &mut self.enable, I2C_IRQ_MASK, "I²C INT_ENA"),
            (0x2c, _) => ro(val, self.raw & self.enable),
            (0x30 | 0x34, _) => checked_rw(
                val,
                &mut self.sda[((at - 0x30) / 4) as usize],
                0x1ff,
                "I²C SDA HOLD/SAMPLE",
            ),
            (0x38, _) => checked_rw(val, &mut self.high, 0xffff, "I²C SCL_HIGH_PERIOD"),
            (0x40..=0x4c, _) if at % 4 == 0 => checked_rw(
                val,
                &mut self.setup[((at - 0x40) / 4) as usize],
                0x1ff,
                "I²C START/STOP",
            ),
            (0x50, _) => checked_rw(val, &mut self.filter, 0x3ff, "I²C FILTER_CFG"),
            (0x54, _) => checked_rw(val, &mut self.clock, 0x3f_ffff, "I²C CLK_CONF"),
            (0x58..=0x74, _) if at % 4 == 0 => checked_rw(
                val,
                &mut self.command[((at - 0x58) / 4) as usize],
                CMD_BITS | CMD_DONE,
                "I²C COMD",
            ),
            (0x80, Some(value)) if value & !0x3f == 0 => {
                // i2c_ll_master_clr_bus: число импульсов из SCL_RST_SLV_NUM.
                // Граница пакетной модели: выходное время счётчика, без проверки SDA.
                self.clear_until = if value & 1 != 0 {
                    Some(
                        now + source_to_cpu(
                            u64::from(value >> 1) * self.timing()?.1,
                            u64::from(XTAL_HZ),
                            cpu,
                        ),
                    )
                } else {
                    None
                };
                self.scl_sp = value;
                Ok(Some(0))
            }
            (0x80, None) => ro(val, self.scl_sp),
            (0x80, _) => Err(unsupported("I²C: принудительный уровень SDA/SCL")),
            _ => Ok(None),
        }?;
        Ok((result, None))
    }

    fn reply(
        &mut self,
        now: u64,
        id: u64,
        status: TransferStatus,
        bytes: Vec<u8>,
    ) -> Result<(), BoardFault> {
        let pending = self
            .pending
            .as_mut()
            .filter(|p| p.reply.is_none() && now < p.timeout_at.unwrap_or(p.response_due))
            .ok_or_else(|| invalid("I²C: нет открытого окна ответа или ответ опоздал"))?;
        let request = pending
            .request
            .as_ref()
            .filter(|req| req.id == id)
            .ok_or_else(|| invalid("I²C: ответ не соответствует запросу"))?;
        let expected = match status {
            TransferStatus::Complete => request.read_len,
            TransferStatus::Nack if pending.checks_ack => 0,
            TransferStatus::NoResponse if pending.timeout_at.is_some() => 0,
            _ => return Err(invalid("I²C: недопустимый статус ответа")),
        };
        if bytes.len() != expected {
            return Err(invalid("I²C: неверная длина ответа"));
        }
        pending.reply = Some((now, Reply { status, bytes }));
        Ok(())
    }

    fn finish(&mut self, now: u64, cancelled: bool) -> Option<OutputEvent> {
        if self.nack_stop_at.is_some_and(|at| now >= at || cancelled) {
            self.nack_stop_at = None;
            self.link = None;
            if !cancelled {
                self.raw |= I2C_COMPLETE;
            }
        }
        if self.clear_until.is_some_and(|at| now >= at || cancelled) {
            self.clear_until = None;
            self.scl_sp &= !1; // I2C_SCL_RST_SLV_EN, самосброс.
        }
        let pending = self.pending.as_ref()?;
        let stretch = pending
            .reply
            .as_ref()
            .map_or(0, |(at, _)| at.saturating_sub(pending.response_due));
        let (deadline, status) = if cancelled {
            (now, TransferStatus::Cancelled)
        } else {
            match (
                pending.request.is_some(),
                pending.reply.as_ref().map(|(_, reply)| reply.status),
            ) {
                (false, _) | (_, Some(TransferStatus::Complete)) => {
                    (pending.nominal_end + stretch, TransferStatus::Complete)
                }
                (_, Some(TransferStatus::Nack)) if pending.timeout_at.is_some() => {
                    (pending.response_due + stretch, TransferStatus::Nack)
                }
                _ if pending.timeout_at.is_none() => (pending.nack_complete, TransferStatus::Nack),
                _ => (pending.timeout_at?, TransferStatus::NoResponse),
            }
        };
        if now < deadline {
            return None;
        }
        let pending = self.pending.take()?;
        self.nack = status == TransferStatus::Nack;
        let mut bytes = Vec::new();
        match status {
            TransferStatus::Complete => {
                for cmd in &mut self.command[..pending.commands] {
                    *cmd |= CMD_DONE;
                }
                if pending.terminal == END {
                    self.link = Some(pending.link);
                    self.raw |= I2C_END;
                } else {
                    self.link = None;
                    self.raw |= I2C_COMPLETE;
                }
                if let Some((_, reply)) = pending.reply {
                    bytes = reply.bytes;
                    self.rx.extend(bytes.iter().copied());
                }
            }
            TransferStatus::Nack => {
                // TRM §28.5 и s_i2c_send_commands: NACK останавливает WRITE.
                // При END следующий STOP задаёт драйвер, при STOP он уже есть в списке.
                self.command[0] |= CMD_DONE;
                self.raw |= I2C_NACK;
                if pending.timeout_at.is_none() {
                    // Совместимость с прежним ELF: STOP автоматически после NACK адреса.
                    self.raw |= I2C_COMPLETE;
                    self.link = None;
                } else {
                    if pending.terminal == STOP {
                        let stop_at = pending.nack_complete + stretch;
                        if stop_at > now {
                            self.link = Some(pending.link);
                            self.nack_stop_at = Some(stop_at);
                        } else {
                            self.link = None;
                            self.raw |= I2C_COMPLETE;
                        }
                    } else {
                        self.link = Some(pending.link);
                    }
                }
            }
            TransferStatus::NoResponse => {
                self.raw |= I2C_TIMEOUT;
                self.link = None;
            }
            TransferStatus::Cancelled => self.link = None,
        }
        let request = pending.request?;
        Some(OutputEvent {
            cycle: deadline,
            output: BoardOutput::I2cDone {
                bus: request.bus,
                id: request.id,
                address: request.address,
                status,
                bytes,
            },
        })
    }
}

#[derive(Clone, Default)]
struct Spi {
    control: u32,
    clock: u32,
    clock_gate: u32,
    user: u32,
    user1: u32,
    user2: u32,
    address: u32,
    length: u32,
    misc: u32,
    words: [u32; SPI_BYTES / 4],
    raw: u32,
    enable: u32,
    pending: Option<(SpiRequest, Option<Reply>)>,
    dma_config: u32,
}

impl Spi {
    fn chip_select(&self) -> Result<u8, BoardFault> {
        // spi_reg.h: SPI_CS0_DIS…SPI_CS5_DIS, активный CS выбирается в MISC.
        let enabled = !self.misc & 0x3f;
        if enabled.count_ones() != 1 {
            return Err(invalid("SPI2 требует ровно один активный CS"));
        }
        Ok(enabled.trailing_zeros() as u8)
    }

    fn start(
        &mut self,
        now: u64,
        id: u64,
        timing: SpiTiming,
        dma: Option<(&mut EspDma, &mut EspMemory)>,
    ) -> Result<SpiRequest, BoardFault> {
        const DATA_FULL_DUPLEX: u32 = (1 << 28) | (1 << 27) | 1;
        if self.pending.is_some() {
            return Err(invalid("SPI2 уже выполняет обмен"));
        }
        if self.clock_gate & 3 != 3 || self.user & DATA_FULL_DUPLEX != DATA_FULL_DUPLEX {
            return Err(unsupported("SPI2 требует тактирование и полный дуплекс"));
        }
        let chip_select = self.chip_select()?;
        let bits = self.length + 1;
        if !bits.is_multiple_of(8) || self.dma_config & DMA_TX == 0 && bits as usize > SPI_BYTES * 8
        {
            return Err(unsupported("SPI2: требуются целые байты доступного буфера"));
        }
        if !matches!(self.dma_config & DMA_DUPLEX, 0 | DMA_DUPLEX) {
            return Err(unsupported(
                "SPI2: в полном дуплексе нужны оба направления GDMA",
            ));
        }
        // ESP-IDF spi_ll_master_set_mode: второе поле — полярность фронта вывода.
        let mode = match (self.misc & (1 << 29) != 0, self.user & (1 << 9) != 0) {
            (false, false) => SpiMode::Mode0,
            (false, true) => SpiMode::Mode1,
            (true, true) => SpiMode::Mode2,
            (true, false) => SpiMode::Mode3,
        };
        let order = |mask| {
            if self.control & mask == 0 {
                BitOrder::MsbFirst
            } else {
                BitOrder::LsbFirst
            }
        };
        // SPI_CLK_GATE.MST_CLK_SEL, TRM рег. 27.13.
        let source = u64::from(if self.clock_gate & 4 == 0 {
            XTAL_HZ
        } else {
            PLL80
        });
        let source_per_bit = if self.clock >> 31 != 0 {
            1
        } else {
            let n = (self.clock >> 12 & 63) + 1;
            let high = (self.clock >> 6 & 63) + 1;
            let low = (self.clock & 63) + 1;
            // spi_ll_freq_for_pre_n: high=floor(N/2), low=N, включая нечётный N.
            if n < 2 || high != n / 2 || low != n {
                return Err(unsupported(
                    "SPI2 CLOCK: неподдержанное соотношение делителей",
                ));
            }
            u64::from(n * ((self.clock >> 18 & 15) + 1))
        };
        // TRM §27.6.1: 0.5·T при выключенном поле, (значение+1.5)·T при включённом.
        // Между первым и последним фронтом захвата (N−1) периодов.
        // Старые ELF используют прежний срок с добавочным полупериодом.
        let setup = if self.user & (1 << 7) == 0 {
            1
        } else {
            2 * (self.user1 >> 17 & 31) + 3
        };
        let hold = if self.user & (1 << 6) == 0 {
            1
        } else {
            2 * (self.user1 >> 22 & 31) + 3
        };
        let half_periods = u64::from(2 * (bits - 1) + setup + hold + u32::from(timing.legacy));
        let length = bits as usize / 8;
        let write = if self.dma_config & DMA_TX != 0 {
            let (dma, memory) =
                dma.ok_or_else(|| unsupported("SPI2 GDMA: SRAM не передана с шины CPU"))?;
            let mut write = Vec::with_capacity(length);
            for i in 0..length {
                let beat = dma
                    .transmit(memory, SPI2_DMA)?
                    .ok_or_else(|| invalid("SPI2 GDMA: TX не дал очередной байт"))?;
                if (beat.eof == EspDmaEof::Success) != (i + 1 == length) {
                    return Err(invalid("SPI2 GDMA: TX EOF не совпадает с длиной SPI"));
                }
                write.push(beat.byte);
            }
            write
        } else {
            self.words
                .iter()
                .flat_map(|word| word.to_le_bytes())
                .take(length)
                .collect()
        };
        let request = SpiRequest {
            bus: 2,
            id,
            chip_select,
            mode,
            write_order: order(1 << 26),
            read_order: order(1 << 25),
            write,
            cycles_per_bit: source_to_cpu(source_per_bit, source, timing.cpu),
            complete_cycle: now
                + source_to_cpu(half_periods * source_per_bit, 2 * source, timing.cpu),
        };
        self.pending = Some((request.clone(), None));
        Ok(request)
    }

    fn access(
        &mut self,
        at: u32,
        val: Option<u32>,
        now: u64,
        id: u64,
        timing: SpiTiming,
        dma: Option<(&mut EspDma, &mut EspMemory)>,
    ) -> Result<(Option<u32>, Option<SpiRequest>), BoardFault> {
        if val.is_some() && self.pending.is_some() && !matches!(at, 0x34 | 0x38 | 0x3c) {
            return Err(invalid("изменение SPI2 во время транзакции"));
        }
        let result = match (at, val) {
            (0, Some(value)) if value & !(SPI_USR | SPI_UPDATE) == 0 => {
                if value & SPI_USR != 0 {
                    return Ok((Some(0), Some(self.start(now, id, timing, dma)?)));
                }
                Ok(Some(0))
            }
            (0, Some(_)) => Err(unsupported("SPI2 CMD")),
            (0, None) => ro(val, u32::from(self.pending.is_some()) * SPI_USR),
            (4, _) => super::rw(val, &mut self.address, u32::MAX), // SPI_ADDR.
            (8, _) => checked_rw(val, &mut self.control, 0x063c_0000, "SPI2 CTRL"),
            (0xc, _) => checked_rw(val, &mut self.clock, 0x803f_ffff, "SPI2 CLOCK"),
            (0x10, _) => checked_rw(
                val,
                &mut self.user,
                0x1800_02c1,
                "SPI2 USER: обычный полный дуплекс",
            ),
            // Драйвер записывает длины −1 даже при выключенных фазах.
            // USER не допускает включения COMMAND/ADDR/DUMMY в пакетном режиме.
            (0x14, _) => checked_rw(val, &mut self.user1, 0xffff_00ff, "SPI2 USER1"),
            (0x18, _) => checked_rw(val, &mut self.user2, 0xf800_ffff, "SPI2 USER2"),
            (0x1c, _) => checked_rw(val, &mut self.length, 0x3ffff, "SPI2 MS_DLEN"),
            (0x20, _) => checked_rw(
                val,
                &mut self.misc,
                0x2000_003f,
                "SPI2 MISC: активный низкий CS",
            ),
            (0x34, _) => checked_rw(val, &mut self.enable, SPI_COMPLETE, "SPI2 DMA_INT_ENA"),
            (0x38, _) => command(val, |bits| self.raw &= !bits),
            // SPI_TRANS_DONE_INT_RAW: spi_ll_{set,clear}_int_stat.
            (0x3c, _) => checked_rw(val, &mut self.raw, SPI_COMPLETE, "SPI2 DMA_INT_RAW"),
            (0x40, _) => ro(val, self.raw & self.enable),
            (0x98..=0xd4, _) => {
                super::rw(val, &mut self.words[((at - 0x98) / 4) as usize], u32::MAX)
            }
            (0x30, _) => {
                // В мастер-режиме при пустой очереди AFIFO сбрасывать нечего.
                checked_rw(
                    val.map(|v| v & !DMA_FIFO_RESET),
                    &mut self.dma_config,
                    DMA_DUPLEX | DMA_SLAVE_CLEAR,
                    "SPI2 DMA_CONF",
                )
            }
            // spi_reg.h: SPI_{DIN_MODE,DIN_NUM,DOUT_MODE} с нулевой компенсацией.
            (0x24 | 0x28 | 0x2c, None | Some(0)) => Ok(Some(0)),
            (0x24 | 0x28 | 0x2c, _) => Err(unsupported("SPI2: компенсация задержки")),
            (0xe0, None | Some(0)) => Ok(Some(0)), // SPI_SLAVE: мастер.
            (0xe0, Some(_)) => Err(unsupported("SPI2 ведомый режим")),
            (0xe8, _) => checked_rw(val, &mut self.clock_gate, 7, "SPI2 CLK_GATE"),
            _ => Ok(None),
        }?;
        Ok((result, None))
    }

    fn reply(
        &mut self,
        now: u64,
        id: u64,
        status: TransferStatus,
        bytes: Vec<u8>,
    ) -> Result<(), BoardFault> {
        let (request, reply) = self
            .pending
            .as_mut()
            .filter(|(req, reply)| req.id == id && reply.is_none() && now < req.complete_cycle)
            .ok_or_else(|| {
                invalid("SPI2: ответ не соответствует ожидаемому запросу или опоздал")
            })?;
        match status {
            TransferStatus::Complete if bytes.len() == request.write.len() => {}
            TransferStatus::NoResponse if bytes.is_empty() => {}
            _ => return Err(invalid("SPI2: неверный статус или длина ответа")),
        }
        *reply = Some(Reply { status, bytes });
        Ok(())
    }

    fn finish(
        &mut self,
        now: u64,
        cancelled: bool,
        dma: Option<(&mut EspDma, &mut EspMemory)>,
    ) -> Result<Option<OutputEvent>, BoardFault> {
        let Some((request, _)) = self.pending.as_ref() else {
            return Ok(None);
        };
        if !cancelled && now < request.complete_cycle {
            return Ok(None);
        }
        let (request, reply) = self.pending.take().unwrap();
        let (status, bytes) = if cancelled {
            (TransferStatus::Cancelled, Vec::new())
        } else if let Some(reply) = reply {
            (reply.status, reply.bytes)
        } else {
            (TransferStatus::NoResponse, Vec::new())
        };
        if status == TransferStatus::Complete {
            if self.dma_config & DMA_RX != 0 {
                let (dma, memory) =
                    dma.ok_or_else(|| unsupported("SPI2 GDMA: SRAM не передана с шины CPU"))?;
                for (i, &byte) in bytes.iter().enumerate() {
                    let eof = if i + 1 == bytes.len() {
                        EspDmaEof::Success
                    } else {
                        EspDmaEof::None
                    };
                    if !dma.receive(memory, SPI2_DMA, EspDmaBeat { byte, eof })? {
                        return Err(invalid("SPI2 GDMA: RX не принял очередной байт"));
                    }
                }
            } else {
                for (i, &byte) in bytes.iter().enumerate() {
                    let shift = (i % 4) * 8;
                    self.words[i / 4] =
                        self.words[i / 4] & !(255 << shift) | (u32::from(byte) << shift);
                }
            }
            self.raw |= SPI_COMPLETE;
        }
        Ok(Some(OutputEvent {
            cycle: if cancelled {
                now
            } else {
                request.complete_cycle
            },
            output: BoardOutput::SpiDone {
                bus: request.bus,
                id: request.id,
                chip_select: request.chip_select,
                status,
                bytes,
            },
        }))
    }
}

#[derive(Clone, Default)]
pub(super) struct EspBuses {
    i2c: I2c,
    spi: Spi,
    pub requests: Vec<OutputEvent>,
    pub completed: Vec<OutputEvent>,
    pub next_id: u64,
    pub fault: Option<BoardFault>,
}

impl EspBuses {
    pub fn active(&self) -> bool {
        self.i2c_active() || self.spi_active()
    }

    pub fn i2c_active(&self) -> bool {
        self.i2c.pending.is_some()
            || self.i2c.link.is_some()
            || self.i2c.clear_until.is_some()
            || self.i2c.nack_stop_at.is_some()
    }

    pub fn spi_active(&self) -> bool {
        self.spi.pending.is_some()
    }

    pub fn spi_chip_select(&self) -> Result<u8, BoardFault> {
        self.spi.chip_select()
    }

    pub fn reset_io(&mut self, i2c: bool, spi: bool, now: u64) {
        if i2c {
            self.completed.extend(self.i2c.finish(now, true));
            self.i2c = I2c::default();
        }
        if spi {
            let event = self
                .spi
                .finish(now, true, None)
                .expect("отменённый SPI не обращается к памяти");
            self.completed.extend(event);
            self.spi = Spi::default();
        }
    }

    pub fn spi_uses_dma(&self) -> bool {
        self.spi.dma_config & DMA_DUPLEX != 0
    }

    pub fn spi_dma_active(&self) -> bool {
        self.spi_uses_dma() && self.spi.pending.is_some()
    }

    pub(super) fn access(
        &mut self,
        bus: u32,
        at: u32,
        val: Option<u32>,
        now: u64,
        cpu: CpuRate,
        legacy: bool,
        dma: Option<(&mut EspDma, &mut EspMemory)>,
    ) -> Access {
        let id = self.next_id;
        let (result, output) = match bus {
            0 => {
                let (result, req) = self
                    .i2c
                    .access(at, val, now, id, &mut self.completed, cpu)?;
                (result, req.map(BoardOutput::I2c))
            }
            _ => {
                let (result, req) =
                    self.spi
                        .access(at, val, now, id, SpiTiming { cpu, legacy }, dma)?;
                (result, req.map(BoardOutput::Spi))
            }
        };
        if let Some(output) = output {
            self.next_id += 1; // Для каждого запроса исполняется хотя бы один такт (< 2^53).
            self.requests.push(OutputEvent { cycle: now, output });
        }
        Ok(result)
    }

    pub fn reply(&mut self, now: u64, input: BoardInput) -> Result<(), BoardFault> {
        match input {
            BoardInput::I2cReply {
                bus: 0,
                id,
                status,
                bytes,
            } => self.i2c.reply(now, id, status, bytes),
            BoardInput::SpiReply {
                bus: 2,
                id,
                status,
                bytes,
            } => self.spi.reply(now, id, status, bytes),
            _ => Err(unsupported("номер контроллера I²C/SPI")),
        }
    }

    pub fn tick(&mut self, now: u64, cancelled: bool) {
        self.tick_with_dma(now, cancelled, None);
    }

    pub(super) fn tick_with_dma(
        &mut self,
        now: u64,
        cancelled: bool,
        dma: Option<(&mut EspDma, &mut EspMemory)>,
    ) {
        self.completed.extend(self.i2c.finish(now, cancelled));
        let event = match self.spi.finish(now, cancelled, dma) {
            Ok(event) => event,
            Err(fault) => {
                self.fault = Some(fault);
                None
            }
        };
        if let Some(event) = event {
            if matches!(
                event.output,
                BoardOutput::SpiDone {
                    status: TransferStatus::NoResponse,
                    ..
                }
            ) {
                self.fault = Some(invalid("SPI2: обмен без подтверждённых входных данных"));
            }
            self.completed.push(event);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // TRM §2.4, §27.5 и gdma_reg.h. Адреса ОЗУ и данные выбраны для опыта.
    const GDMA: u32 = 0x6003_f000;
    const TX_FIRST: u32 = 0x3fc8_0400;
    const TX_LAST: u32 = TX_FIRST + 12; // Три 32-битных слова на дескриптор.
    const RX_DESC: u32 = 0x3fc8_0500;
    const TX_BUF: u32 = 0x3fc8_0600;
    const RX_BUF: u32 = 0x3fc8_0700;
    const PAYLOAD: usize = SPI_BYTES + 2;
    const OWNER: u32 = 1 << 31;
    const EOF: u32 = 1 << 30;
    const CHECK_OWNER: u32 = 1 << 12;
    const RX_DONE_EOF: u32 = (1 << 1) | 1;

    fn wr(dma: &mut EspDma, offset: u32, value: u32) {
        assert_eq!(
            dma.access(0, GDMA + offset, 4, Some(value)).unwrap(),
            Some(0)
        );
    }

    fn rd(dma: &mut EspDma, offset: u32) -> u32 {
        dma.access(0, GDMA + offset, 4, None).unwrap().unwrap()
    }

    fn desc(memory: &mut EspMemory, addr: u32, flags: u32, buffer: u32, next: u32) {
        for (offset, value) in [(0, flags), (4, buffer), (8, next)] {
            memory.write(0, addr + offset, 4, value).unwrap();
        }
    }

    fn fixture(rx_ready: bool) -> (EspBuses, EspDma, EspMemory, Vec<u8>) {
        let mut memory = EspMemory::default();
        let mut dma = EspDma::default();
        let mut buses = EspBuses::default();
        let payload: Vec<_> = (0..PAYLOAD as u8).collect();
        let first = SPI_BYTES as u32;
        let last = PAYLOAD as u32 - first;
        memory.load_ram_segment(TX_BUF, &payload).unwrap();
        memory.load_ram_segment(RX_BUF, &[0xff; PAYLOAD]).unwrap();
        desc(
            &mut memory,
            TX_FIRST,
            OWNER | (first << 12) | first,
            TX_BUF,
            TX_LAST,
        );
        desc(
            &mut memory,
            TX_LAST,
            OWNER | EOF | (last << 12) | last,
            TX_BUF + first,
            0,
        );
        desc(
            &mut memory,
            RX_DESC,
            if rx_ready { OWNER } else { 0 } | PAYLOAD as u32,
            RX_BUF,
            0,
        );
        dma.set_bus_control(true, false);
        // IN_CONF1, OUT_CONF1.CHECK_OWNER; IN/OUT_PERI_SEL; IN/OUT_LINK.START.
        wr(&mut dma, 0x74, CHECK_OWNER);
        wr(&mut dma, 0xd4, CHECK_OWNER);
        wr(&mut dma, 0xa0, u32::from(SPI2_DMA));
        wr(&mut dma, 0x100, u32::from(SPI2_DMA));
        wr(&mut dma, 0x80, (1 << 22) | (RX_DESC & 0xfffff));
        wr(&mut dma, 0xe0, (1 << 21) | (TX_FIRST & 0xfffff));
        wr(&mut dma, 0x08, RX_DONE_EOF); // INT_ENA.
        for (offset, value) in [
            (0xe8, 3),                      // CLK_GATE: XTAL, CLK_EN и MST_CLK_ACTIVE.
            (0x0c, 1 << 31),                // CLOCK: EQU_SYSCLK.
            (0x10, 0x1800_0001),            // USER: MISO, MOSI, DOUTDIN.
            (0x1c, PAYLOAD as u32 * 8 - 1), // MS_DLEN: число бит − 1.
            (0x20, 0x3e),                   // MISC: CS0.
            (0x30, DMA_DUPLEX),
            (0x34, SPI_COMPLETE),
        ] {
            assert_eq!(
                buses.access(2, offset, Some(value), 0, (XTAL_HZ, 1), false, None),
                Ok(Some(0))
            );
        }
        (buses, dma, memory, payload)
    }

    #[test]
    fn spi_gdma_цепочка_tx_ответ_rx_irq_и_отмена() {
        let (mut buses, mut dma, mut memory, payload) = fixture(true);
        buses
            .access(
                2,
                0,
                Some(SPI_USR),
                0,
                (XTAL_HZ, 1),
                false,
                Some((&mut dma, &mut memory)),
            )
            .unwrap();
        let [request] = buses.requests.as_slice() else {
            panic!("ожидался один запрос SPI")
        };
        let BoardOutput::Spi(req) = &request.output else {
            panic!("{request:?}")
        };
        assert_eq!(req.write, payload);
        let at = req.complete_cycle;
        assert_eq!(at, PAYLOAD as u64 * 8); // TRM §27.6: (N−1) T + T/2 + T/2.
        assert_eq!(rd(&mut dma, 0xe8), TX_LAST); // OUT_EOF_DES_ADDR.
        let bytes: Vec<_> = payload.into_iter().rev().collect();
        buses
            .reply(
                1,
                BoardInput::SpiReply {
                    bus: 2,
                    id: req.id,
                    status: TransferStatus::Complete,
                    bytes: bytes.clone(),
                },
            )
            .unwrap();
        let (mut cancelled, mut cdma, mut cmem) = (buses.clone(), dma.clone(), memory.clone());
        cancelled.tick_with_dma(at - 1, true, Some((&mut cdma, &mut cmem)));
        assert!(matches!(
            cancelled.completed[0].output,
            BoardOutput::SpiDone {
                status: TransferStatus::Cancelled,
                ..
            }
        ));
        assert_eq!(cmem.read(0, RX_BUF, 1), Ok(0xff));
        assert_eq!(rd(&mut cdma, 0x04), 0);

        buses.tick_with_dma(at - 1, false, Some((&mut dma, &mut memory)));
        assert!(buses.completed.is_empty());
        assert_eq!(memory.read(0, RX_BUF, 1), Ok(0xff));
        buses.tick_with_dma(at, false, Some((&mut dma, &mut memory)));
        assert_eq!(buses.fault, None);
        for (i, byte) in bytes.into_iter().enumerate() {
            assert_eq!(memory.read(0, RX_BUF + i as u32, 1), Ok(u32::from(byte)));
        }
        assert_eq!(
            memory.read(0, RX_DESC, 4),
            Ok(EOF | (PAYLOAD as u32) << 12 | PAYLOAD as u32)
        );
        assert_eq!(rd(&mut dma, 0x88), RX_DESC); // IN_SUC_EOF_DES_ADDR.
        assert_eq!(rd(&mut dma, 0x04), RX_DONE_EOF); // INT_ST.
        assert_eq!(
            buses.access(2, 0x40, None, at, (XTAL_HZ, 1), false, None),
            Ok(Some(SPI_COMPLETE))
        );
        wr(&mut dma, 0x0c, RX_DONE_EOF);
        assert_eq!(rd(&mut dma, 0x04), 0);
    }

    #[test]
    fn spi_gdma_ошибка_приёмника_не_публикует_успешное_завершение() {
        let (mut buses, mut dma, mut memory, payload) = fixture(false);
        buses
            .access(
                2,
                0,
                Some(SPI_USR),
                0,
                (XTAL_HZ, 1),
                false,
                Some((&mut dma, &mut memory)),
            )
            .unwrap();
        let BoardOutput::Spi(req) = &buses.requests[0].output else {
            panic!("ожидался SPI")
        };
        let end = req.complete_cycle;
        buses
            .reply(
                1,
                BoardInput::SpiReply {
                    bus: 2,
                    id: req.id,
                    status: TransferStatus::Complete,
                    bytes: payload,
                },
            )
            .unwrap();
        buses.tick_with_dma(end, false, Some((&mut dma, &mut memory)));
        assert!(buses.fault.is_some());
        assert!(buses.completed.is_empty());
        assert_eq!(memory.read(0, RX_BUF, 1), Ok(0xff));
        assert_eq!(rd(&mut dma, 0x04), 0); // INT_ST: нет RX_DONE и RX_EOF.
        assert_ne!(rd(&mut dma, 0) & (1 << 5), 0); // DMA_IN_DSCR_ERR_INT_RAW.
        assert_eq!(
            buses.access(2, 0x40, None, end, (XTAL_HZ, 1), false, None),
            Ok(Some(0))
        );
    }
}
