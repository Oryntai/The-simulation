//! Пакетный адаптер прошивки C3. Его ABI не занимает регистров чипа.
//! Заглушка верхнего драйвера размещает команду в DRAM, выполняет custom-0,
//! за которым лежит 32-битный адрес команды. Следующая инструкция через 8 байт.
//! Порядок слов и коды определены в публичном EspRadioAbi.

use crate::memory::Bus;
use crate::{
    BoardFault, BoardOutput, EspRadioAbi as Abi, EspRadioPort, InputOutcome, OutputEvent, Packet,
    RadioStatus,
};

#[derive(Clone, Default)]
struct Port {
    open: bool,
    incoming: Option<Packet>,
    incoming_reply: Option<u64>,
    received_reply: Option<u64>,
    outgoing: Option<(u64, RadioStatus)>,
}

#[derive(Clone, Default)]
pub(super) struct Driver {
    ports: [Port; EspRadioPort::EspNow as usize + 1],
    pub(super) events: Vec<OutputEvent>,
    pub(super) request: bool,
}

impl Driver {
    pub(super) fn listening(&self, port: u32) -> bool {
        self.ports.get(port as usize).is_some_and(|p| p.open)
    }

    pub(super) fn reply_id(&mut self, port: u32, id: u64) -> Result<InputOutcome, BoardFault> {
        if port_kind(port)? != EspRadioPort::EspNow {
            return Err(invalid("ответ приложения требует порт ESP-NOW"));
        }
        let slot = &mut self.ports[port as usize];
        if slot.incoming.is_none() || slot.incoming_reply.is_some() {
            return Err(invalid("нет принятого пакета для номера ответа"));
        }
        slot.incoming_reply = Some(id);
        Ok(InputOutcome::Applied)
    }

    pub(super) fn receive(
        &mut self,
        port: u32,
        packet: Packet,
    ) -> Result<InputOutcome, BoardFault> {
        let kind = port_kind(port)?;
        check_length(kind, packet.bytes.len())?;
        let slot = &mut self.ports[port as usize];
        if !slot.open || slot.incoming.is_some() {
            return Ok(InputOutcome::PacketDropped);
        }
        slot.incoming = Some(packet);
        Ok(InputOutcome::Applied)
    }

    pub(super) fn result(
        &mut self,
        cycle: u64,
        port: u32,
        sent_cycle: u64,
        status: RadioStatus,
    ) -> Result<InputOutcome, BoardFault> {
        let kind = port_kind(port)?;
        if status == RadioStatus::Pending
            || (kind != EspRadioPort::EspNow
                && matches!(
                    status,
                    RadioStatus::MacConfirmed
                        | RadioStatus::BroadcastSent
                        | RadioStatus::ApplicationConfirmed
                        | RadioStatus::ApplicationTimedOut
                ))
        {
            return Err(invalid("неприменимое подтверждение"));
        }
        let slot = &mut self.ports[port as usize];
        if slot.outgoing != Some((sent_cycle, RadioStatus::Pending)) {
            return Ok(InputOutcome::Unchanged);
        }
        slot.outgoing = Some((sent_cycle, status));
        self.emit(
            cycle,
            BoardOutput::RadioStatus {
                port,
                sent_cycle,
                status,
            },
        );
        Ok(InputOutcome::Applied)
    }

    pub(super) fn reset(&mut self, cycle: u64) {
        for (port, slot) in self.ports.iter_mut().enumerate() {
            if let Some((sent_cycle, RadioStatus::Pending)) = slot.outgoing {
                self.events.push(OutputEvent {
                    cycle,
                    output: BoardOutput::RadioStatus {
                        port: port as u32,
                        sent_cycle,
                        status: RadioStatus::NotAnalyzed("передача прервана сбросом или питанием"),
                    },
                });
            }
            *slot = Port::default();
        }
        self.request = false;
    }

    fn emit(&mut self, cycle: u64, output: BoardOutput) {
        self.events.push(OutputEvent { cycle, output });
    }

    pub(super) fn call(&mut self, bus: &mut Bus, pc: u32) -> Result<(), BoardFault> {
        let literal = pc
            .checked_add(Abi::INSTRUCTION_BYTES)
            .ok_or_else(|| invalid("PC"))?;
        // RISC-V IALIGN=16. Inline-слово читается двумя полусловами.
        let address = bus.fetch(literal, 2)? | (bus.fetch(literal + 2, 2)? << u16::BITS);
        let cycle = bus.hw.cycles;
        let mut command = Mailbox { bus, pc, address };
        command.check_range(Abi::PAYLOAD_OFFSET)?;
        if address % size_of::<u32>() as u32 != 0 {
            return Err(invalid("не выровнен блок команды"));
        }
        let op = command.word(Abi::FUNCTION)?;
        let port = command.word(Abi::PORT)?;
        let kind = port_kind(port)?;
        let slot = &mut self.ports[port as usize];

        match op {
            Abi::OPEN => {
                slot.open = true;
                command.set_word(Abi::STATUS, Abi::OK)?;
            }
            Abi::SEND | Abi::RECEIVE | Abi::TX_STATUS | Abi::ACK if !slot.open => {
                return Err(invalid("порт не открыт"));
            }
            Abi::SEND => {
                let len = command.word(Abi::LENGTH)? as usize;
                check_length(kind, len)?;
                command.check_range(Abi::PAYLOAD_OFFSET + len)?;
                if slot
                    .outgoing
                    .is_some_and(|(_, status)| status == RadioStatus::Pending)
                {
                    command.set_word(Abi::STATUS, Abi::BUSY)?;
                    return Ok(());
                }
                let mut bytes = Vec::with_capacity(len);
                for i in 0..len {
                    bytes.push(command.byte(Abi::PAYLOAD_OFFSET + i)?);
                }
                command.set_word(Abi::STATUS, Abi::PENDING)?;
                slot.outgoing = Some((cycle, RadioStatus::Pending));
                self.emit(
                    cycle,
                    BoardOutput::RadioStatus {
                        port,
                        sent_cycle: cycle,
                        status: RadioStatus::Pending,
                    },
                );
                self.emit(
                    cycle,
                    BoardOutput::Radio {
                        port,
                        packet: Packet {
                            bytes,
                            rssi_dbm: None,
                        },
                    },
                );
                self.request = true;
            }
            Abi::RECEIVE => {
                let capacity = command.word(Abi::LENGTH)? as usize;
                check_length(kind, capacity)?;
                command.check_range(Abi::PAYLOAD_OFFSET + capacity)?;
                let Some(packet) = &slot.incoming else {
                    command.set_word(Abi::LENGTH, 0)?;
                    return command.set_word(Abi::STATUS, Abi::EMPTY);
                };
                command.set_word(Abi::LENGTH, packet.bytes.len() as u32)?;
                if packet.bytes.len() > capacity {
                    return command.set_word(Abi::STATUS, Abi::BUFFER_TOO_SMALL);
                }
                let rssi = packet.rssi_dbm.map_or(Abi::RSSI_UNAVAILABLE, i32::from);
                command.set_word(Abi::RSSI, rssi as u32)?;
                for (i, &byte) in packet.bytes.iter().enumerate() {
                    command.set_byte(Abi::PAYLOAD_OFFSET + i, byte)?;
                }
                command.set_word(Abi::STATUS, Abi::OK)?;
                slot.incoming = None;
                slot.received_reply = slot.incoming_reply.take();
            }
            Abi::TX_STATUS => {
                let status = match slot.outgoing.map(|(_, status)| status) {
                    None => Abi::EMPTY,
                    Some(RadioStatus::Pending) => Abi::PENDING,
                    Some(RadioStatus::MacConfirmed) => Abi::MAC_CONFIRMED,
                    Some(RadioStatus::BroadcastSent) => Abi::BROADCAST_SENT,
                    Some(RadioStatus::ApplicationConfirmed) => Abi::APPLICATION_CONFIRMED,
                    Some(RadioStatus::ApplicationTimedOut) => Abi::APPLICATION_TIMED_OUT,
                    Some(RadioStatus::Failed) => Abi::FAILED,
                    Some(RadioStatus::NotAnalyzed(_)) => Abi::NOT_ANALYZED,
                };
                command.set_word(Abi::STATUS, status)?;
            }
            Abi::ACK if kind == EspRadioPort::EspNow => {
                let Some(id) = slot.received_reply.take() else {
                    return command.set_word(Abi::STATUS, Abi::EMPTY);
                };
                command.set_word(Abi::STATUS, Abi::PENDING)?;
                self.emit(cycle, BoardOutput::RadioAck { port, id });
                self.request = true;
            }
            Abi::ACK => return Err(invalid("ответ приложения требует порт ESP-NOW")),
            _ => {
                return Err(BoardFault::Unsupported(format!(
                    "неизвестная функция пакетного драйвера {op:#x}, PC {pc:#010x}"
                )));
            }
        }
        Ok(())
    }
}

fn port_kind(port: u32) -> Result<EspRadioPort, BoardFault> {
    match port {
        p if p == EspRadioPort::Wifi as u32 => Ok(EspRadioPort::Wifi),
        p if p == EspRadioPort::Ble as u32 => Ok(EspRadioPort::Ble),
        p if p == EspRadioPort::EspNow as u32 => Ok(EspRadioPort::EspNow),
        _ => Err(BoardFault::Unsupported(format!("пакетный порт C3 {port}"))),
    }
}

fn check_length(kind: EspRadioPort, len: usize) -> Result<(), BoardFault> {
    if len == 0 || len > kind.max_bytes() {
        return Err(invalid("размер пакета или буфера вне границ порта"));
    }
    Ok(())
}

fn invalid(reason: &str) -> BoardFault {
    BoardFault::Misconfigured(format!("адаптер радио C3: {reason}"))
}

struct Mailbox<'a> {
    bus: &'a mut Bus,
    pc: u32,
    address: u32,
}

impl Mailbox<'_> {
    fn check_range(&self, len: usize) -> Result<(), BoardFault> {
        // TRM v1.4 §3.3.2, табл. 3.3-1, SRAM1 на DBUS. Весь блок
        // проверяется до записи. Доступ к регистрам из адаптера запрещён.
        const SRAM1_DBUS: std::ops::Range<u32> = 0x3fc8_0000..0x3fce_0000;
        if !SRAM1_DBUS.contains(&self.address)
            || self
                .address
                .checked_add(len as u32)
                .is_none_or(|end| end > SRAM1_DBUS.end)
        {
            return Err(invalid("буфер должен целиком лежать в SRAM1 DBUS"));
        }
        Ok(())
    }

    fn word(&mut self, offset: usize) -> Result<u32, BoardFault> {
        self.bus.read(
            self.pc,
            self.address + offset as u32,
            size_of::<u32>() as u8,
        )
    }

    fn set_word(&mut self, offset: usize, value: u32) -> Result<(), BoardFault> {
        self.bus.write(
            self.pc,
            self.address + offset as u32,
            size_of::<u32>() as u8,
            value,
        )
    }

    fn byte(&mut self, offset: usize) -> Result<u8, BoardFault> {
        self.bus
            .read(self.pc, self.address + offset as u32, 1)
            .map(|v| v as u8)
    }

    fn set_byte(&mut self, offset: usize, value: u8) -> Result<(), BoardFault> {
        self.bus
            .write(self.pc, self.address + offset as u32, 1, u32::from(value))
    }
}
