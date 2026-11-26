//! Счётчики частоты RTC_SLOW_CLK, TRM §§11.2.5, 11.3.4 и рег. 11.18–20.

use super::{Access, esp_rtc::RC_SLOW_HZ};
use crate::{
    BoardFault,
    esp_clock::{QUARTERS_PER_SECOND, XTAL_HZ},
};

const CONFIG: u32 = 0x68;
const RESULT: u32 = 0x6c;
const TIMEOUT: u32 = 0x80;
const START: u32 = 1 << 31;
const MAX_SHIFT: u32 = 16;
const MAX_MASK: u32 = 0x7fff << MAX_SHIFT;
const READY: u32 = 1 << 15;
const SELECT_SHIFT: u32 = 13;
const SELECT_MASK: u32 = 3 << SELECT_SHIFT;
const CYCLING: u32 = 1 << 12;
const VALUE_SHIFT: u32 = 7;
const VALUE_VALID: u32 = 1;
const THRESHOLD_SHIFT: u32 = 7;
const THRESHOLD_MASK: u32 = u32::MAX << THRESHOLD_SHIFT;
const TIMEOUT_RESET_COUNT: u32 = 0xf << 3;
const TIMEOUT_FLAG: u32 = 1;
const XTAL_PERIOD: u128 = (QUARTERS_PER_SECOND / XTAL_HZ) as u128;

#[derive(Clone)]
pub(super) struct RtcCalibration {
    config: u32,
    timeout: u32,
    result: u32,
    elapsed: u128,
    // При сбросе TRM выбирает RC_FAST_DIV, цифровой выход которого
    // выключен по RTC_CNTL_DIG_CLK8M_D256_EN. Этот цикл может дать timeout.
    reset_wait: bool,
    pending: bool,
}

impl Default for RtcCalibration {
    fn default() -> Self {
        Self {
            config: (1 << MAX_SHIFT) | (1 << SELECT_SHIFT) | CYCLING,
            timeout: THRESHOLD_MASK | (3 << 3),
            result: 0,
            elapsed: 0,
            reset_wait: true,
            pending: true,
        }
    }
}

impl RtcCalibration {
    pub(super) fn access(&mut self, offset: u32, value: Option<u32>) -> Access {
        let result = match (offset, value) {
            (CONFIG, None) => self.config,
            (RESULT, None) => self.result,
            (TIMEOUT, None) => self.timeout,
            (TIMEOUT, Some(word))
                if word & !(THRESHOLD_MASK | TIMEOUT_RESET_COUNT | TIMEOUT_FLAG) == 0 =>
            {
                if (word ^ self.timeout) & TIMEOUT_RESET_COUNT != 0 {
                    return Err(BoardFault::Unsupported(
                        "TIMG: изменение задержки сброса калибратора".into(),
                    ));
                }
                self.timeout = (self.timeout & TIMEOUT_FLAG) | (word & !TIMEOUT_FLAG);
                0
            }
            (CONFIG, Some(word))
                if word & !(START | MAX_MASK | READY | SELECT_MASK | CYCLING) == 0 =>
            {
                let previous = self.config;
                let start = word & (START | CYCLING) & !previous != 0;
                if start && (word & SELECT_MASK != 0 || word & MAX_MASK == 0) {
                    return Err(BoardFault::Unsupported(
                        "TIMG: калибровка требует RC_SLOW_CLK и ненулевого RTC_CALI_MAX".into(),
                    ));
                }
                if start {
                    self.elapsed = 0;
                    self.pending = true;
                    self.reset_wait = false;
                    self.timeout &= !TIMEOUT_FLAG;
                    self.result &= !VALUE_VALID;
                } else if word & (START | CYCLING) == 0 {
                    self.pending = false;
                    self.reset_wait = false;
                } else if self.pending
                    && !self.reset_wait
                    && (word ^ previous) & (SELECT_MASK | MAX_MASK) != 0
                {
                    return Err(BoardFault::Unsupported(
                        "TIMG: изменение текущей калибровки".into(),
                    ));
                }
                self.config = word & !READY;
                if !start {
                    self.config |= previous & READY;
                }
                0
            }
            _ => return Ok(None),
        };
        Ok(Some(result))
    }

    pub(super) fn advance(&mut self, elapsed: u128) {
        if !self.pending {
            return;
        }
        self.elapsed += elapsed;
        let xtal_limit = u128::from((self.timeout >> THRESHOLD_SHIFT) + 1) * XTAL_PERIOD;
        if !self.reset_wait {
            let count = u128::from((self.config & MAX_MASK) >> MAX_SHIFT);
            let xtal = count * u128::from(XTAL_HZ);
            // Номинальный RC_SLOW. Фаза физических осцилляторов требует замера.
            let completion = (xtal * XTAL_PERIOD).div_ceil(u128::from(RC_SLOW_HZ));
            if self.elapsed >= completion && completion <= xtal_limit {
                self.result = ((xtal / u128::from(RC_SLOW_HZ)) as u32) << VALUE_SHIFT;
                if self.config & CYCLING != 0 {
                    self.result |= VALUE_VALID;
                    self.elapsed %= completion;
                } else {
                    self.config |= READY;
                    self.pending = false;
                }
                return;
            }
        }
        if self.elapsed >= xtal_limit {
            self.timeout |= TIMEOUT_FLAG;
            self.pending = false;
        }
    }
}
