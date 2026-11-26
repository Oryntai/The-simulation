//! Настройка принудительного питания памяти в покое.

use super::Access;
use crate::{BoardFault, esp_reset_profile::EspResetValues};

// ESP-IDF system_reg.h, syscon_reg.h и port/esp32c3/rtc_sleep.c:
// адрес, маска полей R/W и значение сброса.
const FRONT_END_MEM: u32 = 0x6002_609c; // SYSCON_FRONT_END_MEM_PD_REG.
const FRONT_END_FORCE_PU: u32 = (1 << 4) | (1 << 2) | 1;
const KNOWN: [(u32, u32, u32); 7] = [
    (0x600c_000c, 1, 1), // SYSTEM_MEM_PD_MASK_REG.
    (
        FRONT_END_MEM,
        FRONT_END_FORCE_PU | (FRONT_END_FORCE_PU << 1),
        FRONT_END_FORCE_PU,
    ),
    (0x6002_60ac, 0x3f, 0x3f), // SYSCON_MEM_POWER_UP_REG, SRAM + ROM.
    (0x6002_6014, u32::MAX, 0xfffc_e030), // SYSCON_WIFI_CLK_EN_REG.
    (0x6002_6018, u32::MAX, 0), // SYSCON_WIFI_RST_EN_REG.
    (0x600c_0020, 0xfff, 255), // SYSTEM_BT_LPCK_DIV_INT_REG.
    (0x600c_0024, 0x1fff_ffff, (1 << 25) | (1 << 12) | 1), // SYSTEM_BT_LPCK_DIV_FRAC_REG.
];

// ESP-IDF reg_base.h, bb_reg.h, nrx_reg.h, fe_reg.h: адрес и маска
// FORCE_PD; соответствующий FORCE_PU находится на один бит выше.
// В этих заголовках слово сброса не указано.
const RADIO: [(u32, &str, u32); 4] = [
    (0x6001_d054, "BBPD_CTRL", (1 << 2) | 1),
    (
        0x6001_ccd4,
        "NRXPD_CTRL",
        (1 << 6) | (1 << 4) | (1 << 2) | 1,
    ),
    (0x6000_6090, "FE_GEN_CTRL", 1 << 4),
    (0x6000_50f0, "FE2_TX_INTERP_CTRL", 1 << 9),
];

pub(super) fn supports_reset_value(address: u32, field: &str, value: u32) -> bool {
    RADIO
        .iter()
        .any(|&(addr, name, pd)| addr == address && name == field && value & !(pd | (pd << 1)) == 0)
}

#[derive(Clone)]
pub(super) struct EspSleepRegisters {
    known: [u32; KNOWN.len()],
    radio: [Option<u32>; RADIO.len()],
}

impl Default for EspSleepRegisters {
    fn default() -> Self {
        Self {
            known: KNOWN.map(|(_, _, reset)| reset),
            radio: [None; RADIO.len()],
        }
    }
}

impl EspSleepRegisters {
    pub(super) fn access(
        &mut self,
        pc: u32,
        addr: u32,
        size: u8,
        value: Option<u32>,
        resets: &mut EspResetValues,
    ) -> Access {
        if let Some((index, &(_, field, pd))) =
            RADIO.iter().enumerate().find(|(_, row)| row.0 == addr)
        {
            let unknown = || BoardFault::UnsupportedRegister {
                pc,
                addr,
                size,
                write: value.is_some(),
                block: "питание памяти радио",
            };
            if size as usize != size_of::<u32>() {
                return Err(unknown());
            }
            if let Some(word) = value {
                if word & !(pd | (pd << 1)) != 0 {
                    return Err(unknown());
                }
                if word & (word >> 1) & pd != 0 {
                    return Err(BoardFault::Misconfigured(
                        "противоположные FORCE_PU/PD памяти радио".into(),
                    ));
                }
                self.radio[index] = Some(word);
                return Ok(Some(0));
            }
            let word = match self.radio[index] {
                Some(word) => Some(word),
                None => resets.read(addr, field)?,
            };
            return word.map(Some).ok_or_else(unknown);
        }
        let Some((index, &(_, mask, _))) = KNOWN.iter().enumerate().find(|(_, row)| row.0 == addr)
        else {
            return Ok(None);
        };
        if size as usize != size_of::<u32>() || value.is_some_and(|word| word & !mask != 0) {
            return Err(BoardFault::UnsupportedRegister {
                pc,
                addr,
                size,
                write: value.is_some(),
                block: "SYSREG/SYSCON питания памяти",
            });
        }
        if addr == FRONT_END_MEM
            && value.is_some_and(|word| word & (word >> 1) & FRONT_END_FORCE_PU != 0)
        {
            return Err(BoardFault::Misconfigured(
                "противоположные FORCE_PU/PD памяти SYSCON".into(),
            ));
        }
        // rtc_sleep_pu записывает правила перехода. Значения не выключают
        // SRAM/ROM в активном режиме, физическое время перехода не вычисляется.
        super::rw(value, &mut self.known[index], mask)
    }
}
