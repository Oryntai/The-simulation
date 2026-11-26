//! Настройка цифрового SAR и температурного датчика, TRM §§34.2–34.3.
//! Чтение результата преобразования здесь остаётся явным отказом.

use super::Access;
use crate::BoardFault;

// ESP-IDF soc/esp32c3/register/soc/{reg_base,apb_saradc_reg,system_reg}.h.
pub(super) const BASE: u32 = 0x6004_0000;
pub(super) const ADC_GATE: u32 = 1 << 28;
pub(super) const TSENS_GATE: u32 = 1 << 10;
const CTRL: u32 = 0x00;
const CTRL2: u32 = 0x04;
const FSM_WAIT: u32 = 0x0c;
const PATT1: u32 = 0x18;
const PATT2: u32 = 0x1c;
const DMA: u32 = 0x50;
const CLOCK: u32 = 0x54;
const TSENS: u32 = 0x58;
const TSENS2: u32 = 0x5c;

// apb_saradc_reg.h: смещение, маска полей R/W, слово холодного сброса.
const REGISTERS: [(u32, u32, u32); 9] = [
    (
        CTRL,
        (3 << 30) | (3 << 27) | (1 << 23) | (0xfff << 6) | 3,
        (1 << 30) | (7 << 15) | (4 << 7) | (1 << 6),
    ),
    (CTRL2, (0x1fff << 12) | 0x7ff, (10 << 12) | (255 << 1)),
    (FSM_WAIT, 0x00ff_ffff, (255 << 16) | (8 << 8) | 8),
    (PATT1, 0x00ff_ffff, 0),
    (PATT2, 0x00ff_ffff, 0),
    (DMA, (3 << 30) | 0xffff, 255),
    (CLOCK, 0x007f_ffff, 4),
    (TSENS, 0x3ff << 13, 6 << 14),
    (TSENS2, 0xffff, (1 << 14) | 2),
];

#[derive(Clone)]
pub(super) struct EspSarDigital([u32; REGISTERS.len()]);

impl Default for EspSarDigital {
    fn default() -> Self {
        Self(REGISTERS.map(|(_, _, reset)| reset))
    }
}

impl EspSarDigital {
    pub(super) fn reset_tsens(&mut self) {
        for (index, &(offset, _, reset)) in REGISTERS.iter().enumerate() {
            if matches!(offset, TSENS | TSENS2) {
                self.0[index] = reset;
            }
        }
    }

    pub(super) fn access(
        &mut self,
        pc: u32,
        addr: u32,
        size: u8,
        value: Option<u32>,
        out_of_reset: bool,
    ) -> Access {
        let unknown = || BoardFault::UnsupportedRegister {
            pc,
            addr,
            size,
            write: value.is_some(),
            block: "APB_SARADC raw flash",
        };
        if size as usize != size_of::<u32>() || !out_of_reset {
            return Err(unknown());
        }
        let offset = addr - BASE;
        let Some((index, &(_, mask, _))) = REGISTERS
            .iter()
            .enumerate()
            .find(|(_, row)| row.0 == offset)
        else {
            return Err(unknown());
        };
        if let Some(word) = value {
            if word & !mask != 0 {
                return Err(unknown());
            }
            if offset == CTRL && word & (1 << 1) != 0 {
                return Err(BoardFault::Unsupported(
                    "SAR: непосредственный запуск без модели преобразования".into(),
                ));
            }
            self.0[index] = word;
            return Ok(Some(0));
        }
        if offset == TSENS
            && (self.0[index] & (1 << 22) != 0 || self.0[index + 1] & (3 << 12) == (3 << 12))
        {
            return Err(BoardFault::Unsupported(
                "TSENS: нет результата преобразования температуры".into(),
            ));
        }
        Ok(Some(self.0[index]))
    }
}
