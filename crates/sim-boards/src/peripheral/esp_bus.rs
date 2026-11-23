//! ESP32-C3 TRM, главы I²C и SPI. Регистры также сверены с esp-idf v5.4.2.

use super::{Access, checked_rw, command, invalid, ro, unsupported};
use crate::{
    BitOrder, BoardFault, BoardInput, BoardOutput, I2cRequest, OutputEvent, SpiMode, SpiRequest,
    TransferStatus,
};
use std::collections::VecDeque;

pub(super) const I2C_FIFO: usize = 32; // ESP32-C3 TRM, I²C FIFO.
pub(super) const SPI_BYTES: usize = 64; // W0…W15, 32-битные слова.
const I2C_NACK: u32 = 1 << 10;
const I2C_COMPLETE: u32 = 1 << 7;
const SPI_COMPLETE: u32 = 1 << 12;

#[derive(Clone)]
struct Reply {
    status: TransferStatus,
    bytes: Vec<u8>,
}

#[derive(Clone)]
struct I2cPending {
    request: I2cRequest,
    commands: usize,
    nack_complete: u64,
    reply: Option<Reply>,
}

#[derive(Clone, Default)]
struct I2c {
    low: u32,
    high: u32,
    control: u32,
    clock: u32,
    setup: [u32; 4],
    fifo_conf: u32,
    command: [u32; 8],
    tx: VecDeque<u8>,
    rx: VecDeque<u8>,
    raw: u32,
    enable: u32,
    nack: bool,
    pending: Option<I2cPending>,
}

impl I2c {
    fn start(&mut self, now: u64, id: u64) -> Result<I2cRequest, BoardFault> {
        if self.pending.is_some() {
            return Err(invalid("I²C уже выполняет транзакцию"));
        }
        if self.control & 0x110 != 0x110 || self.clock & !0xff != 0x20_0000 {
            return Err(unsupported(
                "I²C требует ведущий режим, MSB и целый делитель XTAL",
            ));
        }
        let command_error = || unsupported("последовательность команд ведущего I²C");
        let mut commands = self.command.iter().map(|&cmd| cmd & 0x3fff);
        // RESTART, WRITE с проверкой ACK, READ с NACK последнего байта, STOP.
        if commands.next() != Some(6 << 11) {
            return Err(command_error());
        }
        let mut command_count = 2;
        let write_count = |command: u32| {
            let count = (command & 255) as usize;
            // WRITE, ACK_CHECK_EN, ожидаемый ACK = 0.
            (command >> 8 == 0b1001 && count > 0).then_some(count)
        };
        let mut first_len = commands
            .next()
            .and_then(write_count)
            .ok_or_else(command_error)?;
        let address_byte = *self.tx.front().ok_or_else(command_error)?;
        let address = address_byte >> 1;
        if !(0x08..=0x77).contains(&address) {
            return Err(unsupported("зарезервированный или 10-битный адрес I²C"));
        }
        let mut next = commands.next();
        while let Some(extra) = next.and_then(write_count) {
            first_len += extra;
            command_count += 1;
            next = commands.next();
        }
        if first_len > self.tx.len() {
            return Err(command_error());
        }
        let write: Vec<_> = self
            .tx
            .iter()
            .skip(1)
            .take(first_len - 1)
            .copied()
            .collect();
        let repeated_start = next == Some(6 << 11);
        let address_bytes = if repeated_start {
            if address_byte & 1 != 0
                || self.tx.get(first_len) != Some(&(address_byte | 1))
                || commands.next() != Some((1 << 11) | (1 << 8) | 1)
            {
                return Err(command_error());
            }
            command_count += 2;
            next = commands.next();
            2
        } else {
            1
        };

        let receives = repeated_start || address_byte & 1 != 0;
        if address_byte & 1 != 0 && first_len != 1 {
            return Err(command_error());
        }
        let mut read_len = 0usize;
        let mut last_nack = false;
        if receives {
            while let Some(read) = next.filter(|cmd| cmd >> 11 == 3) {
                let length = (read & 255) as usize;
                let nack = read & (1 << 10) != 0;
                if length == 0 || last_nack || (nack && length != 1) || read & 0x300 != 0 {
                    return Err(command_error());
                }
                read_len += length;
                last_nack = nack;
                command_count += 1;
                next = commands.next();
            }
            if !last_nack {
                return Err(command_error());
            }
        }
        if next != Some(2 << 11)
            || self.rx.len() + read_len > I2C_FIFO
            || self.tx.len() != write.len() + address_bytes
        {
            return Err(invalid("I²C: STOP, длина FIFO или команды не согласованы"));
        }
        command_count += 1;

        // HAL ESP-IDF: low = raw + 1; high = high + wait_high.
        // XTAL 40 МГц, CPU 80 МГц в данном профиле.
        let clock_div = 2 * u64::from((self.clock & 255) + 1);
        let high = (self.high & 511) + (self.high >> 9);
        if high == 0 || self.low == 0 {
            return Err(invalid("I²C: не задан период SCL"));
        }
        let cycles_per_bit = u64::from(self.low + 1 + high) * clock_div;
        // SCL_START_HOLD, SCL_RSTART_SETUP, SCL_STOP_HOLD и SCL_STOP_SETUP.
        let setup = self.setup.map(|value| u64::from(value + 1) * clock_div);
        let address_ack_cycle = now + setup[0] + 9 * cycles_per_bit;
        let stop = setup[2] + setup[3];
        let complete_cycle = now
            + setup[0]
            + u64::from(repeated_start) * (setup[0] + setup[1])
            + 9 * (write.len() + address_bytes + read_len) as u64 * cycles_per_bit
            + stop;
        let request = I2cRequest {
            bus: 0,
            id,
            address,
            write,
            read_len,
            repeated_start,
            cycles_per_bit,
            address_ack_cycle,
            complete_cycle,
        };
        for cmd in &mut self.command {
            *cmd &= 0x3fff;
        }
        self.tx.clear();
        self.nack = false;
        self.pending = Some(I2cPending {
            request: request.clone(),
            commands: command_count,
            nack_complete: address_ack_cycle + stop,
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
    ) -> Result<(Option<u32>, Option<I2cRequest>), BoardFault> {
        if val.is_some() && self.pending.is_some() && !matches!(at, 0x24 | 0x28) {
            return Err(invalid("изменение I²C во время транзакции"));
        }
        let result = match (at, val) {
            (0, _) => checked_rw(val, &mut self.low, 0x1ff, "I²C SCL_LOW_PERIOD"),
            (4, Some(value)) => {
                // Master, CLK_EN, open-drain, RX-full level, CONF_UPGATE и TRANS_START.
                let mask = 0xb3f;
                if value & !mask != 0 {
                    return Err(unsupported("I²C CTR: разрядность, направление или режим"));
                }
                let old = self.control;
                self.control = value & !0x820;
                if value & 0x20 != 0 {
                    return match self.start(now, id) {
                        Ok(request) => Ok((Some(0), Some(request))),
                        Err(error) => {
                            self.control = old;
                            Err(error)
                        }
                    };
                }
                Ok(Some(0))
            }
            (4, None) => ro(val, self.control),
            (8, _) => ro(
                val,
                ((self.tx.len() as u32) << 18)
                    | ((self.rx.len() as u32) << 8)
                    | (u32::from(self.pending.is_some()) << 4)
                    | u32::from(self.nack),
            ),
            (0x18, Some(value)) if value & !0x73ff == 0 => {
                if value & (1 << 13) != 0 {
                    self.tx.clear();
                }
                if value & (1 << 12) != 0 {
                    self.rx.clear();
                }
                self.fifo_conf = value & 0x43ff;
                Ok(Some(0))
            }
            (0x18, None) => ro(val, self.fifo_conf),
            (0x18, _) => Err(unsupported("I²C FIFO_CONF: поддерживается только FIFO")),
            (0x1c, Some(byte)) if byte <= 255 && self.tx.len() < I2C_FIFO => {
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
            (0x28, _) => checked_rw(
                val,
                &mut self.enable,
                I2C_NACK | I2C_COMPLETE,
                "I²C INT_ENA",
            ),
            (0x2c, _) => ro(val, self.raw & self.enable),
            (0x38, _) => checked_rw(val, &mut self.high, 0x3ffff, "I²C SCL_HIGH_PERIOD"),
            (0x40..=0x4c, _) => checked_rw(
                val,
                &mut self.setup[((at - 0x40) / 4) as usize],
                0x1ff,
                "I²C START/STOP",
            ),
            (0x54, _) => checked_rw(val, &mut self.clock, 0x20_00ff, "I²C CLK_CONF"),
            (0x58..=0x74, _) => {
                let cmd = &mut self.command[((at - 0x58) / 4) as usize];
                if val.is_some() {
                    checked_rw(val, cmd, 0x3fff, "I²C COMD")
                } else {
                    ro(val, *cmd)
                }
            }
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
            .filter(|p| {
                p.request.id == id && p.reply.is_none() && now < p.request.address_ack_cycle
            })
            .ok_or_else(|| invalid("I²C: ответ не соответствует ожидаемому запросу или опоздал"))?;
        let expected = match status {
            TransferStatus::Complete => pending.request.read_len,
            TransferStatus::Nack => 0,
            _ => return Err(invalid("I²C: недопустимый статус ответа")),
        };
        if bytes.len() != expected {
            return Err(invalid("I²C: неверная длина ответа"));
        }
        pending.reply = Some(Reply { status, bytes });
        Ok(())
    }

    fn finish(&mut self, now: u64, cancelled: bool) -> Option<OutputEvent> {
        let pending = self.pending.as_ref()?;
        let ack = pending
            .reply
            .as_ref()
            .is_some_and(|reply| reply.status == TransferStatus::Complete);
        let deadline = if ack {
            pending.request.complete_cycle
        } else {
            pending.nack_complete
        };
        if !cancelled && now < deadline {
            return None;
        }
        let pending = self.pending.take()?;
        let (status, bytes) = if cancelled {
            (TransferStatus::Cancelled, Vec::new())
        } else if ack {
            for cmd in &mut self.command[..pending.commands] {
                *cmd |= 1 << 31;
            }
            self.raw |= I2C_COMPLETE;
            let bytes = pending.reply?.bytes;
            self.rx.extend(bytes.iter().copied());
            (TransferStatus::Complete, bytes)
        } else {
            self.command[0] |= 1 << 31;
            self.raw |= I2C_NACK | I2C_COMPLETE;
            self.nack = true;
            (TransferStatus::Nack, Vec::new())
        };
        Some(OutputEvent {
            cycle: if cancelled { now } else { deadline },
            output: BoardOutput::I2cDone {
                bus: pending.request.bus,
                id: pending.request.id,
                address: pending.request.address,
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
    length: u32,
    misc: u32,
    words: [u32; SPI_BYTES / 4],
    raw: u32,
    enable: u32,
    pending: Option<(SpiRequest, Option<Reply>)>,
}

impl Spi {
    fn start(&mut self, now: u64, id: u64) -> Result<SpiRequest, BoardFault> {
        const DATA_FULL_DUPLEX: u32 = (1 << 28) | (1 << 27) | 1;
        if self.pending.is_some() {
            return Err(invalid("SPI2 уже выполняет обмен"));
        }
        if self.clock_gate & 3 != 3 || self.user & DATA_FULL_DUPLEX != DATA_FULL_DUPLEX {
            return Err(unsupported("SPI2 требует тактирование и полный дуплекс"));
        }
        let enabled = !self.misc & 0x3f;
        if enabled.count_ones() != 1 {
            return Err(invalid("SPI2 требует ровно один активный CS"));
        }
        let chip_select = enabled.trailing_zeros() as u8;
        let bits = self.length + 1;
        if !bits.is_multiple_of(8) || bits as usize > SPI_BYTES * 8 {
            return Err(unsupported(
                "SPI2: длина должна быть целым числом байт в буфере",
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
        let cycles_per_source = if self.clock_gate & 4 == 0 { 2 } else { 1 }; // XTAL / PLL80.
        let cycles_per_bit = if self.clock >> 31 != 0 {
            cycles_per_source
        } else {
            let n = (self.clock >> 12 & 63) + 1;
            let high = (self.clock >> 6 & 63) + 1;
            let low = (self.clock & 63) + 1;
            if n < 2 || !n.is_multiple_of(2) || high != n / 2 || low != n {
                return Err(unsupported(
                    "SPI2 CLOCK: неподдержанное соотношение делителей",
                ));
            }
            cycles_per_source * u64::from(n * ((self.clock >> 18 & 15) + 1))
        };
        // TRM §27.6.1: 0.5·T при выключенном поле, (значение+1.5)·T при включённом.
        // Между первым и последним фронтом 2·N−1 полупериодов при равных high/low.
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
        let half_periods = u64::from(2 * bits - 1 + setup + hold);
        let write = self
            .words
            .iter()
            .flat_map(|word| word.to_le_bytes())
            .take(bits as usize / 8)
            .collect();
        let request = SpiRequest {
            bus: 2,
            id,
            chip_select,
            mode,
            write_order: order(1 << 26),
            read_order: order(1 << 25),
            write,
            cycles_per_bit,
            complete_cycle: now + (half_periods * cycles_per_bit).div_ceil(2),
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
    ) -> Result<(Option<u32>, Option<SpiRequest>), BoardFault> {
        const USR: u32 = 1 << 24;
        const UPDATE: u32 = 1 << 23;
        if val.is_some() && self.pending.is_some() && !matches!(at, 0x34 | 0x38) {
            return Err(invalid("изменение SPI2 во время транзакции"));
        }
        let result = match (at, val) {
            (0, Some(value)) if value & !(USR | UPDATE) == 0 => {
                if value & USR != 0 {
                    return Ok((Some(0), Some(self.start(now, id)?)));
                }
                Ok(Some(0))
            }
            (0, Some(_)) => Err(unsupported("SPI2 CMD")),
            (0, None) => ro(val, u32::from(self.pending.is_some()) * USR),
            (8, _) => checked_rw(val, &mut self.control, 0x063c_0000, "SPI2 CTRL"),
            (0xc, _) => checked_rw(val, &mut self.clock, 0x803f_ffff, "SPI2 CLOCK"),
            (0x10, _) => checked_rw(
                val,
                &mut self.user,
                0x1800_02c1,
                "SPI2 USER: обычный полный дуплекс",
            ),
            (0x14, _) => checked_rw(val, &mut self.user1, 0x07fe_0000, "SPI2 USER1"),
            (0x1c, _) => checked_rw(val, &mut self.length, 0x3ffff, "SPI2 MS_DLEN"),
            (0x20, _) => checked_rw(
                val,
                &mut self.misc,
                0x2000_003f,
                "SPI2 MISC: активный низкий CS",
            ),
            (0x34, _) => checked_rw(val, &mut self.enable, SPI_COMPLETE, "SPI2 DMA_INT_ENA"),
            (0x38, _) => command(val, |bits| self.raw &= !bits),
            (0x3c, _) => ro(val, self.raw),
            (0x40, _) => ro(val, self.raw & self.enable),
            (0x98..=0xd4, _) => {
                super::rw(val, &mut self.words[((at - 0x98) / 4) as usize], u32::MAX)
            }
            (0x30 | 0xe0, None | Some(0)) => Ok(Some(0)), // DMA и ведомый режим выключены.
            (0x30 | 0xe0, Some(_)) => Err(unsupported("SPI2 DMA или ведомый режим")),
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

    fn finish(&mut self, now: u64, cancelled: bool) -> Option<OutputEvent> {
        let (request, _) = self.pending.as_ref()?;
        if !cancelled && now < request.complete_cycle {
            return None;
        }
        let (request, reply) = self.pending.take()?;
        let (status, bytes) = if cancelled {
            (TransferStatus::Cancelled, Vec::new())
        } else if let Some(reply) = reply {
            (reply.status, reply.bytes)
        } else {
            (TransferStatus::NoResponse, Vec::new())
        };
        if status == TransferStatus::Complete {
            self.raw |= SPI_COMPLETE;
            for (i, &byte) in bytes.iter().enumerate() {
                let shift = (i % 4) * 8;
                self.words[i / 4] =
                    self.words[i / 4] & !(255 << shift) | (u32::from(byte) << shift);
            }
        }
        Some(OutputEvent {
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
        })
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
    pub fn access(&mut self, bus: u32, at: u32, val: Option<u32>, now: u64) -> Access {
        let id = self.next_id;
        let (result, output) = match bus {
            0 => {
                let (result, req) = self.i2c.access(at, val, now, id)?;
                (result, req.map(BoardOutput::I2c))
            }
            _ => {
                let (result, req) = self.spi.access(at, val, now, id)?;
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
        self.completed.extend(self.i2c.finish(now, cancelled));
        if let Some(event) = self.spi.finish(now, cancelled) {
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

    pub fn irq(&self) -> [bool; 2] {
        [
            self.i2c.raw & self.i2c.enable != 0,
            self.spi.raw & self.spi.enable != 0,
        ]
    }
}
