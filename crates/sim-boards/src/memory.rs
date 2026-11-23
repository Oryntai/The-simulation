use crate::peripheral::Hardware;
use crate::{BoardFault, BoardId, image::Image};

#[derive(Clone)]
pub(crate) struct Region {
    base: u32,
    data: Vec<u8>,
    initialized: Vec<bool>,
    writable: bool,
}

impl Region {
    fn new(base: u32, size: usize, writable: bool) -> Self {
        Self {
            base,
            data: vec![if writable { 0 } else { 0xff }; size],
            initialized: vec![false; size],
            writable,
        }
    }

    fn offset(&self, addr: u32, size: usize) -> Option<usize> {
        let offset = addr.checked_sub(self.base)? as usize;
        (offset.checked_add(size)? <= self.data.len()).then_some(offset)
    }
}

#[derive(Clone)]
pub(crate) struct Bus {
    pub hw: Hardware,
    pub id: BoardId,
    regions: Vec<Region>,
}

impl Bus {
    pub fn new(id: BoardId, image: &Image) -> Result<Self, BoardFault> {
        use BoardId::*;
        // RP2040 §2.2 / Pico flash; RM0008 §3 / DS5319; 7810D §7;
        // ESP32-C3 TRM §3: инструкция и данные SRAM разделяют физическую память.
        let mut regions = match id {
            Virtual => unreachable!("виртуальный профиль имеет собственную шину"),
            Rp2040 => vec![
                Region::new(0x1000_0000, 2 << 20, false),
                Region::new(0x2000_0000, 264 << 10, true),
            ],
            Stm32f103c8 => vec![
                Region::new(0x0800_0000, 64 << 10, false),
                Region::new(0x2000_0000, 20 << 10, true),
            ],
            Atmega328p => vec![
                Region::new(0, 32 << 10, false),
                Region::new(0x100, 2 << 10, true),
            ],
            Esp32C3 => vec![
                Region::new(0x4037_c000, 400 << 10, true),
                Region::new(0x3c00_0000, 8 << 20, false),
                Region::new(0x4200_0000, 8 << 20, false),
                Region::new(0x5000_0000, 8 << 10, true),
            ],
        };
        for segment in &image.segments {
            let addr = alias(id, segment.addr);
            let (region, start) = regions
                .iter_mut()
                .find_map(|r| r.offset(addr, segment.data.len()).map(|o| (r, o)))
                .ok_or_else(|| {
                    BoardFault::BadImage(format!(
                        "сегмент по адресу {:#010x} вне памяти платы",
                        segment.addr
                    ))
                })?;
            let end = start + segment.data.len();
            if region.initialized[start..end].iter().any(|x| *x) {
                return Err(BoardFault::BadImage(format!(
                    "перекрывающиеся сегменты по адресу {:#010x}",
                    segment.addr
                )));
            }
            region.data[start..end].copy_from_slice(&segment.data);
            region.initialized[start..end].fill(true);
        }
        Ok(Self {
            hw: Hardware::new(id),
            id,
            regions,
        })
    }

    pub fn fetch(&self, pc: u32, size: u8) -> Result<u32, BoardFault> {
        let addr = alias(self.id, pc);
        for region in &self.regions {
            if let Some(offset) = region.offset(addr, size as usize) {
                if region.initialized[offset..offset + size as usize]
                    .iter()
                    .all(|x| *x)
                {
                    return Ok(read_value(&region.data[offset..], size));
                }
                break;
            }
        }
        Err(BoardFault::BusError {
            pc,
            addr: pc,
            write: false,
        })
    }

    pub fn read(&mut self, pc: u32, addr: u32, size: u8) -> Result<u32, BoardFault> {
        self.access(pc, addr, size, None)
    }

    pub fn write(&mut self, pc: u32, addr: u32, size: u8, value: u32) -> Result<(), BoardFault> {
        self.access(pc, addr, size, Some(value)).map(|_| ())
    }

    fn access(
        &mut self,
        pc: u32,
        addr: u32,
        size: u8,
        value: Option<u32>,
    ) -> Result<u32, BoardFault> {
        if !addr.is_multiple_of(u32::from(size)) {
            return Err(BoardFault::Unaligned { pc, addr, size });
        }
        // Harvard: AVR обращается к flash через LPM; обычный LD идёт в пространство данных.
        if self.id != BoardId::Atmega328p || addr >= 0x100 {
            let mapped = alias(self.id, addr);
            for r in &mut self.regions {
                if self.id == BoardId::Atmega328p && !r.writable {
                    continue;
                }
                if let Some(o) = r.offset(mapped, size as usize) {
                    if let Some(v) = value {
                        if !r.writable {
                            return Err(BoardFault::WriteToReadOnly { pc, addr });
                        }
                        r.data[o..o + size as usize]
                            .copy_from_slice(&v.to_le_bytes()[..size as usize]);
                        r.initialized[o..o + size as usize].fill(true);
                        return Ok(0);
                    }
                    return Ok(read_value(&r.data[o..], size));
                }
            }
        }
        self.hw.access(addr, size, value)?.ok_or_else(|| {
            if (0x4000_0000..0x6000_0000).contains(&addr)
                || (0x6000_0000..0x6010_0000).contains(&addr)
                || (0xd000_0000..0xd001_0000).contains(&addr)
                || (0xe000_0000..0xe010_0000).contains(&addr)
                || (self.id == BoardId::Atmega328p && (0x20..0x100).contains(&addr))
            {
                BoardFault::UnsupportedRegister {
                    pc,
                    addr,
                    write: value.is_some(),
                    size,
                    block: "периферии",
                }
            } else {
                BoardFault::BusError {
                    pc,
                    addr,
                    write: value.is_some(),
                }
            }
        })
    }
}

fn alias(id: BoardId, addr: u32) -> u32 {
    match id {
        BoardId::Stm32f103c8 if addr < 64 << 10 => addr + 0x0800_0000,
        BoardId::Esp32C3 if (0x3fc8_0000..0x3fce_0000).contains(&addr) => {
            addr + (0x4038_0000 - 0x3fc8_0000)
        }
        _ => addr,
    }
}

fn read_value(bytes: &[u8], size: u8) -> u32 {
    let mut value = [0; 4];
    value[..size as usize].copy_from_slice(&bytes[..size as usize]);
    u32::from_le_bytes(value)
}
