//! CPU PMS: память SRAM0/1, ROM, RTC FAST. TRM v1.4 §§14.4–14.5,
//! sensitive_reg.h и memprot_ll.h ESP-IDF v6.1.1.

use std::cell::Cell;
use std::ops::Range;

use crate::{
    BoardFault,
    esp_memory::{IRAM as INTERNAL_IRAM, IROM, ROM1_DBUS, RTC_FAST, SRAM1_DBUS, SRAM1_IBUS},
};

// TRM §§3.3.2, 14.4.2 и soc/memprot_defs.h.
const SRAM0: Range<u32> = INTERNAL_IRAM.start..SRAM1_IBUS;
const IRAM: Range<u32> = SRAM1_IBUS..INTERNAL_IRAM.end;
const BLOCK: u32 = 0x2_0000;

// TRM рег. 14.22–14.36, 14.37, 14.46–14.47, 14.59–14.60;
// sensitive_reg.h также описывает регистры статуса мониторов.
const BASE: u32 = 0x600c_1000;
const SPLIT_LOCK: u32 = 0x90;
const FIRST_SPLIT: u32 = 0x94;
const LAST_SPLIT: u32 = 0xa4;
const IBUS_LOCK: u32 = 0xa8;
const IBUS_USER: u32 = 0xac;
const IBUS_MACHINE: u32 = 0xb0;
const DBUS_LOCK: u32 = 0xc0;
const DBUS_PERM: u32 = 0xc4;
const PIF_LOCK: u32 = 0xd8;
const RTC_SPLIT: u32 = 0xfc;
const RTC_PERM: u32 = 0x100;
const OBSERVERS: [u32; 3] = [0xb4, 0xc8, 0x130];
const IBUS_MASK: u32 = (0b111 << 18) | 0x7fff;
const DBUS_MASK: u32 = (0b1111 << 24) | (0xff << 12) | 0xff;
const RTC_SPLIT_MASK: u32 = (1 << 22) - 1;
const RTC_PERM_MASK: u32 = (1 << 12) - 1;
const SPLIT_MASK: u32 = (0xff << 14) | 0x3f;
const CLEAR: u32 = 1;
const ENABLE: u32 = 2;
const WORD: u8 = size_of::<u32>() as u8;

#[derive(Clone, Copy)]
pub(crate) enum PmsOp {
    Read,
    Write,
    Fetch,
}

impl PmsOp {
    fn bit(self) -> u32 {
        // memprot_defs.h: SRAM R/W/F, RTC FAST W/R/F.
        match self {
            Self::Read => 1,
            Self::Write => 2,
            Self::Fetch => 4,
        }
    }
}

#[derive(Clone, Copy)]
struct Violation {
    address: u32,
    op: PmsOp,
    user: bool,
    size: u8,
}

#[derive(Clone)]
struct Monitor {
    locked: bool,
    control: u32,
    violation: Cell<Option<Violation>>,
}

impl Default for Monitor {
    fn default() -> Self {
        Self {
            locked: false,
            control: ENABLE | CLEAR,
            violation: Cell::new(None),
        }
    }
}

#[derive(Clone)]
pub(crate) struct EspPms {
    permission_locked: [bool; 3],
    split_locked: bool,
    splits: [u32; 5],
    ibus: [u32; 2],
    dbus: u32,
    rtc_split: u32,
    rtc_perm: u32,
    monitors: [Monitor; 3],
}

impl Default for EspPms {
    fn default() -> Self {
        Self {
            permission_locked: [false; 3],
            split_locked: false,
            splits: [0; 5],
            ibus: [IBUS_MASK; 2],
            dbus: DBUS_MASK,
            rtc_split: RTC_SPLIT_MASK,
            rtc_perm: RTC_PERM_MASK,
            monitors: std::array::from_fn(|_| Monitor::default()),
        }
    }
}

impl EspPms {
    pub(crate) fn access(
        &mut self,
        pc: u32,
        addr: u32,
        size: u8,
        val: Option<u32>,
    ) -> Result<Option<u32>, BoardFault> {
        let offset = addr.wrapping_sub(BASE);
        let unknown = || BoardFault::UnsupportedRegister {
            pc,
            addr,
            size,
            write: val.is_some(),
            block: "PMS",
        };
        if !(SPLIT_LOCK..=OBSERVERS[2] + 12).contains(&offset) {
            return Ok(None);
        }
        if size != WORD || !addr.is_multiple_of(u32::from(WORD)) {
            return Err(unknown());
        }

        let result = match offset {
            SPLIT_LOCK => lock(&mut self.split_locked, val),
            IBUS_LOCK => lock(&mut self.permission_locked[0], val),
            DBUS_LOCK => lock(&mut self.permission_locked[1], val),
            PIF_LOCK => lock(&mut self.permission_locked[2], val),
            FIRST_SPLIT..=LAST_SPLIT => {
                let reg = &mut self.splits[((offset - FIRST_SPLIT) / u32::from(WORD)) as usize];
                if let Some(value) = val.filter(|_| !self.split_locked) {
                    split_address(value & SPLIT_MASK)?;
                }
                update(reg, val, SPLIT_MASK, self.split_locked)
            }
            IBUS_MACHINE | IBUS_USER => update(
                &mut self.ibus[usize::from(offset == IBUS_USER)],
                val,
                IBUS_MASK,
                self.permission_locked[0],
            ),
            DBUS_PERM => update(&mut self.dbus, val, DBUS_MASK, self.permission_locked[1]),
            RTC_SPLIT => update(
                &mut self.rtc_split,
                val,
                RTC_SPLIT_MASK,
                self.permission_locked[2],
            ),
            RTC_PERM => update(
                &mut self.rtc_perm,
                val,
                RTC_PERM_MASK,
                self.permission_locked[2],
            ),
            _ => {
                let Some((index, base)) =
                    OBSERVERS.iter().copied().enumerate().find(|&(i, base)| {
                        (base..=base + if i == 0 { 8 } else { 12 }).contains(&offset)
                    })
                else {
                    return Err(unknown());
                };
                let monitor = &mut self.monitors[index];
                match (offset - base, val) {
                    (0, _) => lock(&mut monitor.locked, val),
                    (4, _) => {
                        let result =
                            update(&mut monitor.control, val, ENABLE | CLEAR, monitor.locked);
                        if result.is_some() && monitor.control & CLEAR != 0 {
                            monitor.violation.set(None);
                        }
                        result
                    }
                    (field, None) => Some(
                        monitor
                            .violation
                            .get()
                            .map_or(0, |v| monitor_status(index, field, v)),
                    ),
                    _ => None,
                }
            }
        };
        result.map(Some).ok_or_else(unknown)
    }

    pub(crate) fn interrupts(&self) -> u64 {
        // ETS_CORE0_{IRAM0,DRAM0,PIF}_PMS_INTR_SOURCE, interrupts.h.
        self.monitors.iter().enumerate().fold(0, |bits, (i, m)| {
            bits | (u64::from(m.control == ENABLE && m.violation.get().is_some()) << (56 + i))
        })
    }

    pub(crate) fn check(
        &self,
        user: bool,
        pc: u32,
        addr: u32,
        size: u8,
        op: PmsOp,
    ) -> Result<(), BoardFault> {
        let end = addr.checked_add(u32::from(size) - 1);
        // Все границы PMS кратны слову. Доступ CPU занимает не более слова
        // и потому может пересечь не более одной границы.
        let second = end.filter(|last| last / u32::from(WORD) != addr / u32::from(WORD));
        for at in std::iter::once(addr).chain(second) {
            let Some((index, permissions)) = self.permissions(at, user)? else {
                continue;
            };
            let required = match (index, op) {
                (2, PmsOp::Read) => 2,
                (2, PmsOp::Write) => 1,
                _ => op.bit(),
            };
            if permissions & required == 0 {
                let monitor = &self.monitors[index];
                if monitor.control == ENABLE && monitor.violation.get().is_none() {
                    monitor.violation.set(Some(Violation {
                        address: addr,
                        op,
                        user,
                        size,
                    }));
                }
                return Err(BoardFault::BusError {
                    pc,
                    addr,
                    write: matches!(op, PmsOp::Write),
                });
            }
        }
        Ok(())
    }

    fn permissions(&self, addr: u32, user: bool) -> Result<Option<(usize, u32)>, BoardFault> {
        let ibus = self.ibus[usize::from(user)];
        let dbus = self.dbus >> if user { 12 } else { 0 };
        let area = if IRAM.contains(&addr) {
            (0, ibus >> (3 * self.area(addr - IRAM.start, false)?) & 7)
        } else if SRAM1_DBUS.contains(&addr) {
            (
                1,
                dbus >> (2 * self.area(addr - SRAM1_DBUS.start, true)?) & 3,
            )
        } else if SRAM0.contains(&addr) {
            (0, ibus >> 12 & 7)
        } else if IROM.contains(&addr) {
            (0, ibus >> 18 & 7)
        } else if ROM1_DBUS.contains(&addr) {
            (1, self.dbus >> (24 + 2 * u32::from(user)) & 3)
        } else if RTC_FAST.contains(&addr) {
            let shift = if user { 11 } else { 0 };
            let boundary = (self.rtc_split >> shift & 0x7ff) << 2;
            let field = 2 * u32::from(user) + u32::from(addr - RTC_FAST.start >= boundary);
            (2, self.rtc_perm >> (3 * field) & 7)
        } else {
            return Ok(None);
        };
        Ok(Some(area))
    }

    fn area(&self, offset: u32, data: bool) -> Result<u32, BoardFault> {
        let main = split_address(self.splits[0])?;
        if data && offset < main {
            return Ok(0);
        }
        if !data && offset >= main {
            return Ok(3);
        }
        let shift = if data { 3 } else { 1 };
        let (first, second) = (
            split_address(self.splits[shift])?,
            split_address(self.splits[shift + 1])?,
        );
        // §14.4.2.2: внутренние границы лежат в своей области и могут совпасть.
        if !(first <= second && if data { first >= main } else { second <= main }) {
            return Err(BoardFault::Misconfigured(
                "границы областей SRAM1 PMS".into(),
            ));
        }
        Ok(u32::from(data) + u32::from(offset >= first) + u32::from(offset >= second))
    }
}

// §14.4.2.2: [16:9] адреса в блоке 128 КиБ; cat=0 до границы,
// 1/2 в блоке границы, 3 после. Все cat=0 означают, что каждый блок ниже границы.
fn split_address(word: u32) -> Result<u32, BoardFault> {
    if word == 0 {
        return Ok(3 * BLOCK);
    }
    let categories = [word & 3, word >> 2 & 3, word >> 4 & 3];
    let block = match categories {
        [1 | 2, 3, 3] => 0,
        [0, 1 | 2, 3] => 1,
        [0, 0, 1 | 2] => 2,
        _ => {
            return Err(BoardFault::Misconfigured(
                "категории блоков SRAM1 PMS".into(),
            ));
        }
    };
    Ok(block * BLOCK + ((word >> 14 & 0xff) << 9))
}

fn lock(state: &mut bool, value: Option<u32>) -> Option<u32> {
    if let Some(value) = value {
        if value & !1 != 0 {
            return None;
        }
        *state |= value != 0;
        return Some(0);
    }
    Some(u32::from(*state))
}

fn update(state: &mut u32, value: Option<u32>, mask: u32, locked: bool) -> Option<u32> {
    if let Some(value) = value {
        if value & !mask != 0 {
            return None;
        }
        if !locked {
            *state = value;
        }
        return Some(0);
    }
    Some(*state)
}

fn monitor_status(index: usize, field: u32, v: Violation) -> u32 {
    // sensitive_reg.h: *_PMS_MONITOR_VIOLATE_STATUS_*, memprot_defs.h: базовые адреса.
    let write = u32::from(matches!(v.op, PmsOp::Write));
    let data = u32::from(!matches!(v.op, PmsOp::Fetch));
    let world = u32::from(v.user);
    match (index, field) {
        (0, 8) => ((v.address - 0x4000_0000) >> 2) << 5 | world << 3 | data << 2 | write << 1 | 1,
        (1, 8) => ((v.address - 0x3c00_0000) >> 2) << 4 | world << 2 | 1,
        (1, 12) => (((1 << v.size) - 1) << (v.address & 3) & 0xf) << 1 | write,
        (2, 8) => world << 6 | write << 5 | v.size.trailing_zeros() << 2 | data << 1 | 1,
        (2, 12) => v.address,
        _ => unreachable!("карта регистров мониторов PMS"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn разделение_алиасов_sram1_проверяет_тип_доступа_и_всю_операцию() {
        let mut pms = EspPms::default();
        let set = |pms: &mut EspPms, reg, value| {
            assert_eq!(pms.access(0, BASE + reg, WORD, Some(value)), Ok(Some(0)));
        };
        // §14.4.2.2: Block0, категории [2,3,3], граница 0x0800.
        for reg in (FIRST_SPLIT..=LAST_SPLIT).step_by(usize::from(WORD)) {
            set(&mut pms, reg, 0x0001_003e);
        }
        set(&mut pms, IBUS_MACHINE, 0x16d); // RX в трёх областях IBUS, запрет в области DBUS.
        set(&mut pms, DBUS_PERM, 0xfc); // RW в трёх областях DBUS, запрет в области IBUS.
        set(&mut pms, OBSERVERS[1] + 4, ENABLE);
        assert_eq!(pms.check(false, 0, 0x4038_07fe, 2, PmsOp::Fetch), Ok(()));
        assert_eq!(pms.check(false, 0, 0x3fc8_0800, 4, PmsOp::Write), Ok(()));
        assert_eq!(
            pms.check(false, 0, 0x4038_07fe, 4, PmsOp::Fetch),
            Err(BoardFault::BusError {
                pc: 0,
                addr: 0x4038_07fe,
                write: false
            })
        );
        assert!(pms.check(false, 0, 0x3fc8_07ff, 2, PmsOp::Write).is_err());
        assert_eq!(pms.interrupts(), 1 << 57);
        assert_eq!(pms.access(0, BASE + 0xd4, WORD, None), Ok(Some(0x11)));
        assert!(pms.check(true, 0, 0x3fc8_0800, 1, PmsOp::Read).is_err());
        set(&mut pms, OBSERVERS[1] + 4, CLEAR | ENABLE);
        assert_eq!(pms.interrupts(), 0);

        set(&mut pms, SPLIT_LOCK, 1);
        set(&mut pms, SPLIT_LOCK, 0);
        set(&mut pms, FIRST_SPLIT, 0);
        assert_eq!(
            pms.access(0, BASE + FIRST_SPLIT, WORD, None),
            Ok(Some(0x0001_003e))
        );
        assert!(pms.access(0, BASE + FIRST_SPLIT, 2, None).is_err());
        assert!(pms.access(0, BASE + 0xd0, WORD, Some(0)).is_err());
    }

    #[test]
    fn rtc_fast_применяет_права_и_не_снимает_блокировку() {
        let mut pms = EspPms::default();
        // §14.4.2.3: 0x10 слов до границы, RX ниже и RW выше.
        assert_eq!(
            pms.access(0, BASE + RTC_SPLIT, WORD, Some(0x10)),
            Ok(Some(0))
        );
        assert_eq!(
            pms.access(0, BASE + RTC_PERM, WORD, Some(0x1e)),
            Ok(Some(0))
        );
        assert_eq!(pms.access(0, BASE + PIF_LOCK, WORD, Some(1)), Ok(Some(0)));
        assert_eq!(
            pms.access(0, BASE + RTC_PERM, WORD, Some(0xfff)),
            Ok(Some(0))
        );
        assert_eq!(pms.check(false, 0, 0x5000_003c, 4, PmsOp::Fetch), Ok(()));
        assert_eq!(pms.check(false, 0, 0x5000_0040, 4, PmsOp::Write), Ok(()));
        assert!(pms.check(false, 0, 0x5000_0040, 2, PmsOp::Fetch).is_err());
        assert!(pms.check(false, 0, 0x5000_003c, 4, PmsOp::Write).is_err());
    }
}
