use crate::esp_memory::EspMemory;
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
    pub(crate) fn new(base: u32, size: usize, writable: bool) -> Self {
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

    pub(crate) fn contains(&self, addr: u32, size: usize) -> bool {
        self.offset(addr, size).is_some()
    }

    pub(crate) fn load(&mut self, addr: u32, data: &[u8], original: u32) -> Result<(), BoardFault> {
        let Some(start) = self.offset(addr, data.len()) else {
            return Err(BoardFault::BadImage(format!(
                "сегмент по адресу {original:#010x} вне памяти платы"
            )));
        };
        let end = start + data.len();
        if self.initialized[start..end].iter().any(|x| *x) {
            return Err(BoardFault::BadImage(format!(
                "перекрывающиеся сегменты по адресу {original:#010x}"
            )));
        }
        self.data[start..end].copy_from_slice(data);
        self.initialized[start..end].fill(true);
        Ok(())
    }

    pub(crate) fn read(&self, addr: u32, size: u8, initialized_only: bool) -> Option<u32> {
        let offset = self.offset(addr, size.into())?;
        let end = offset + usize::from(size);
        if initialized_only && !self.initialized[offset..end].iter().all(|x| *x) {
            return None;
        }
        Some(read_value(&self.data[offset..end], size))
    }

    pub(crate) fn write(&mut self, addr: u32, size: u8, value: u32) -> Option<()> {
        let offset = self.offset(addr, size.into())?;
        let end = offset + usize::from(size);
        self.data[offset..end].copy_from_slice(&value.to_le_bytes()[..size as usize]);
        self.initialized[offset..end].fill(true);
        Some(())
    }
}

#[derive(Clone)]
pub(crate) struct Bus {
    pub hw: Hardware,
    pub id: BoardId,
    pub esp_memory: Option<EspMemory>,
    regions: Vec<Region>,
}

impl Bus {
    pub fn new(id: BoardId, image: &Image) -> Result<Self, BoardFault> {
        use BoardId::*;
        // RP2040 §2.2 / Pico flash; RM0008 §3 / DS5319; 7810D §7.
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
            // TRM §3.3.3.1, табл. 3.3-2. Прежнее отображение сегментов ELF.
            // Отображение физической flash через cache/MMU относится к Т2.5.
            Esp32C3 => vec![
                Region::new(0x3c00_0000, 8 << 20, false),
                Region::new(0x4200_0000, 8 << 20, false),
            ],
        };
        let mut esp_memory = (id == Esp32C3).then(EspMemory::legacy_elf);
        for segment in &image.segments {
            let addr = alias(id, segment.addr);
            if let Some(region) = regions
                .iter_mut()
                .find(|r| r.contains(addr, segment.data.len()))
            {
                region.load(addr, &segment.data, segment.addr)?;
            } else if let Some(memory) = &mut esp_memory {
                memory.load_ram_segment(segment.addr, &segment.data)?;
            } else {
                return Err(BoardFault::BadImage(format!(
                    "сегмент по адресу {:#010x} вне памяти платы",
                    segment.addr
                )));
            }
        }
        Ok(Self {
            hw: Hardware::new(id),
            id,
            esp_memory,
            regions,
        })
    }

    pub fn fetch(&self, pc: u32, size: u8) -> Result<u32, BoardFault> {
        if let Some(memory) = &self.esp_memory
            && !self.regions.iter().any(|r| r.contains(pc, 1))
        {
            return memory.fetch(pc, size);
        }
        let addr = alias(self.id, pc);
        if let Some(region) = self.regions.iter().find(|r| r.contains(addr, size.into()))
            && let Some(value) = region.read(addr, size, true)
        {
            return Ok(value);
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
        check_data_alignment(pc, addr, size)?;
        if let Some(memory) = &mut self.esp_memory
            && let Some(result) = memory.access(pc, addr, size, value)?
        {
            return Ok(result);
        }
        // Harvard: AVR обращается к flash через LPM; обычный LD идёт в пространство данных.
        if self.id != BoardId::Atmega328p || addr >= 0x100 {
            let mapped = alias(self.id, addr);
            for r in &mut self.regions {
                if self.id == BoardId::Atmega328p && !r.writable {
                    continue;
                }
                if r.contains(mapped, size.into()) {
                    if let Some(v) = value {
                        if !r.writable {
                            return Err(BoardFault::WriteToReadOnly { pc, addr });
                        }
                        r.write(mapped, size, v);
                        return Ok(0);
                    }
                    return Ok(r.read(mapped, size, false).unwrap());
                }
            }
        }
        self.hw.access(addr, size, value)?.ok_or_else(|| {
            if (self.id != BoardId::Esp32C3 && (0x4000_0000..0x6000_0000).contains(&addr))
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
        _ => addr,
    }
}

pub(crate) fn check_data_alignment(pc: u32, addr: u32, size: u8) -> Result<(), BoardFault> {
    if !matches!(size, 1 | 2 | 4) {
        return Err(BoardFault::Unsupported(format!(
            "ширина доступа к памяти {size} байт по адресу {addr:#010x}"
        )));
    }
    if !addr.is_multiple_of(u32::from(size)) {
        return Err(BoardFault::Unaligned { pc, addr, size });
    }
    Ok(())
}

pub(crate) fn read_value(bytes: &[u8], size: u8) -> u32 {
    let mut value = [0; 4];
    value[..size as usize].copy_from_slice(&bytes[..size as usize]);
    u32::from_le_bytes(value)
}
