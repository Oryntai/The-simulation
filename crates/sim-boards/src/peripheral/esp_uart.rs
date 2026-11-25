//! UART ESP32-C3 на границе кадров. TRM v1.4 §26.4–26.7 и
//! ESP-IDF v6.1.1: soc/esp32c3/register/soc/uart_reg.h, esp_hal_uart/esp32c3/include/hal/uart_ll.h.

use crate::{BoardFault, InputOutcome};
use std::collections::VecDeque;

// uart_reg.h: UART_INT_{RAW,ST,ENA,CLR}; при кадровом входе нет ошибок линии.
const RX_FULL: u32 = 1 << 0;
const TX_EMPTY: u32 = 1 << 1;
const PARITY_ERROR: u32 = 1 << 2;
const FRAME_ERROR: u32 = 1 << 3;
const RX_OVERFLOW: u32 = 1 << 4;
const BREAK: u32 = 1 << 7;
const RX_TIMEOUT: u32 = 1 << 8;
const TX_DONE: u32 = 1 << 14;
const WAKEUP: u32 = 1 << 19;
const DRIVER_IRQ: u32 = RX_FULL
    | TX_EMPTY
    | PARITY_ERROR
    | FRAME_ERROR
    | RX_OVERFLOW
    | BREAK
    | RX_TIMEOUT
    | TX_DONE
    | WAKEUP;

// uart_ll.h: UART_LL_FIFO_DEF_LEN. Входная очередь стенда тоже ограничена одним FIFO.
const FIFO_LEN: usize = 128;
// uart_reg.h: поля UART_CONF0.
const MEM_CLK_EN: u32 = 1 << 28;
const TX_FIFO_RESET: u32 = 1 << 18;
const RX_FIFO_RESET: u32 = 1 << 17;
// uart_reg.h: поля UART_CONF1.
const RX_TIMEOUT_EN: u32 = 1 << 21;
const THRESHOLD_MASK: u32 = 0x1ff;
// uart_reg.h: поля UART_CLK_CONF.
const RX_RESET: u32 = 1 << 27;
const TX_RESET: u32 = 1 << 26;
const RX_CLK_EN: u32 = 1 << 25;
const TX_CLK_EN: u32 = 1 << 24;
const CORE_RESET: u32 = 1 << 23;
const SCLK_EN: u32 = 1 << 22;
// uart_reg.h: поля UART_ID.
const UPDATE: u32 = 1 << 31;
const HIGH_SPEED: u32 = 1 << 30;

#[derive(Clone, Copy, PartialEq, Eq)]
struct Config {
    divider: u32,
    clock: u32,
    format: u32,
    thresholds: u32,
    memory: u32,
}

impl Default for Config {
    fn default() -> Self {
        // uart_reg.h: reset UART_CLKDIV, CLK_CONF, CONF0, CONF1 и MEM_CONF.
        Self {
            divider: 0x2b6,
            clock: 0x0370_1000,
            format: MEM_CLK_EN | 0x1c,
            thresholds: 96 << 9 | 96,
            memory: 10 << 16 | 0x12,
        }
    }
}

#[derive(Clone, Copy)]
struct Frame {
    start: u128,
    end: u128,
    phase_end: u128,
    timeout: Option<u128>,
    byte: u8,
}

/// Кадровая модель одного UART. Временные метки заданы в единицах одного
/// монотонного времени, одинаковых для MMIO, входа и продвижения.
#[derive(Clone)]
pub struct EspUart {
    ticks_hz: u32,
    apb_hz: u32,
    xtal_hz: u32,
    rc_fast_hz: Option<u32>,
    shadow: Config,
    active: Config,
    id: u32,
    idle: u32,
    filter: u32,
    enable: u32,
    latched: u32,
    rx: VecDeque<u8>,
    wire: VecDeque<Frame>,
    rx_deadline: Option<u128>,
    tx: VecDeque<Frame>,
    done: Vec<(u64, u8)>,
    now: u128,
}

fn invalid(message: &str) -> BoardFault {
    BoardFault::Misconfigured(message.into())
}

fn unsupported(message: &str) -> BoardFault {
    BoardFault::Unsupported(message.into())
}

impl EspUart {
    pub fn new(
        ticks_hz: u32,
        apb_hz: u32,
        xtal_hz: u32,
        rc_fast_hz: Option<u32>,
    ) -> Result<Self, BoardFault> {
        if ticks_hz == 0 || apb_hz == 0 || xtal_hz == 0 || rc_fast_hz == Some(0) {
            return Err(invalid("UART: нулевая частота"));
        }
        let shadow = Config::default();
        Ok(Self {
            ticks_hz,
            apb_hz,
            xtal_hz,
            rc_fast_hz,
            shadow,
            active: shadow,
            id: HIGH_SPEED | 0x500,    // UART_ID_REG reset, uart_reg.h.
            idle: 0x100 << 10 | 0x100, // UART_IDLE_CONF_REG reset, uart_reg.h.
            filter: 0,                 // UART_RX_FILT_REG reset, uart_reg.h.
            enable: 0,
            latched: 0,
            rx: VecDeque::new(),
            wire: VecDeque::new(),
            rx_deadline: None,
            tx: VecDeque::new(),
            done: Vec::new(),
            now: 0,
        })
    }

    /// Период полубита как точная дробь в единицах времени модели.
    fn half_bit(&self, tx: bool) -> Result<(u128, u128), BoardFault> {
        let cfg = self.active;
        let (gate, reset) = if tx {
            (TX_CLK_EN, TX_RESET)
        } else {
            (RX_CLK_EN, RX_RESET)
        };
        if cfg.format & MEM_CLK_EN == 0
            || cfg.memory & (1 << 26) != 0
            || cfg.clock & (gate | SCLK_EN | reset | CORE_RESET) != gate | SCLK_EN
        {
            return Err(invalid(
                "UART: память или тактирование выключено либо активен сброс",
            ));
        }
        let source = match (cfg.clock >> 20) & 3 {
            1 => self.apb_hz,
            2 => self
                .rc_fast_hz
                .ok_or_else(|| unsupported("UART: частота RC_FAST не задана"))?,
            3 => self.xtal_hz,
            _ => return Err(unsupported("UART: неизвестный источник тактирования")),
        };
        // TRM §26.4.1, §26.4.3.1: первый делитель DLL с NUM+1,
        // второй с четырьмя дробными битами. LL драйвера пишет целый первый делитель.
        if cfg.clock & 0xfff != 0 {
            return Err(unsupported("UART: дробный первый делитель"));
        }
        let divider = ((cfg.divider & 0xfff) << 4) | ((cfg.divider >> 20) & 15);
        if divider == 0 {
            return Err(invalid("UART: нулевой делитель линии"));
        }
        let predivider = ((cfg.clock >> 12) & 255) + 1;
        Ok((
            u128::from(self.ticks_hz) * u128::from(divider) * u128::from(predivider),
            32 * u128::from(source),
        ))
    }

    fn frame_half_bits(&self) -> Result<u128, BoardFault> {
        // TRM §26.4.4 и uart_reg.h: START, 5–8 данных, необязательная
        // чётность и STOP 1, 1.5 или 2. Входные байты уже декодированы.
        let format = self.active.format;
        let stop = match format >> 4 & 3 {
            1 => 2,
            2 => 3,
            3 => 4,
            _ => return Err(invalid("UART: зарезервированная длина STOP")),
        };
        let data = 5 + ((format >> 2) & 3);
        let parity = (format >> 1) & 1;
        Ok(u128::from(2 * (1 + data + parity) + stop))
    }

    /// Поместить правильно сформированные кадры на вход линии UART.
    pub fn receive(&mut self, now: u64, bytes: &[u8]) -> Result<InputOutcome, BoardFault> {
        self.tick(now)?;
        let (half_bit, scale) = self.half_bit(false)?;
        let duration = self.frame_half_bits()? * half_bit;
        let width = 5 + ((self.active.format >> 2) & 3); // UART_BIT_NUM, uart_reg.h.
        if bytes.iter().any(|&byte| u32::from(byte) >= 1 << width) {
            return Err(invalid("UART: входной байт шире настроенного кадра"));
        }
        // uart_ll_set_rx_tout: порог UART_MEM_CONF задан в битовых интервалах.
        let threshold = (self.active.memory >> 16) & 0x3ff;
        let timeout = (self.active.thresholds & RX_TIMEOUT_EN != 0)
            .then_some(2 * u128::from(threshold) * half_bit);
        if timeout == Some(0) {
            return Err(invalid("UART: включён тайм-аут с нулевым порогом"));
        }

        let accepted = bytes.len().min(FIFO_LEN - self.wire.len());
        let mut start = self
            .wire
            .back()
            .map_or(self.now * scale, |frame| frame.phase_end);
        let final_timeout =
            (start + accepted as u128 * duration + timeout.unwrap_or(0)).div_ceil(scale);
        if accepted != 0 && final_timeout > u128::from(u64::MAX) {
            return Err(invalid("UART: конец кадра или тайм-аут вне шкалы времени"));
        }
        for &byte in &bytes[..accepted] {
            let end = start + duration;
            self.wire.push_back(Frame {
                start: start.div_ceil(scale),
                end: end.div_ceil(scale),
                phase_end: end,
                timeout: timeout.map(|delay| (end + delay).div_ceil(scale)),
                byte,
            });
            start = end;
        }
        Ok(InputOutcome::Uart {
            accepted,
            dropped: bytes.len() - accepted,
        })
    }

    pub fn tick(&mut self, now: u64) -> Result<(), BoardFault> {
        let now = u128::from(now);
        if now < self.now {
            return Err(invalid("UART: время пошло назад"));
        }
        self.now = now;
        while let Some(frame) = self.wire.front().copied().filter(|frame| frame.end <= now) {
            self.expire_rx(frame.start, Some(frame.start));
            self.wire.pop_front();
            if self.rx.len() == FIFO_LEN {
                self.latched |= RX_OVERFLOW;
            } else {
                self.rx.push_back(frame.byte);
            }
            self.rx_deadline = frame.timeout;
        }
        self.expire_rx(now, self.wire.front().map(|frame| frame.start));
        while let Some(frame) = self.tx.pop_front_if(|frame| frame.end <= now) {
            self.done.push((frame.end as u64, frame.byte));
            if self.tx.is_empty() {
                self.latched |= TX_DONE;
            }
        }
        Ok(())
    }

    fn expire_rx(&mut self, now: u128, next_start: Option<u128>) {
        if let Some(deadline) = self.rx_deadline {
            if next_start.is_some_and(|start| start <= now && start <= deadline) {
                self.rx_deadline = None;
            } else if now >= deadline {
                self.latched |= RX_TIMEOUT;
                self.rx_deadline = None;
            }
        }
    }

    fn tx_fifo_count(&self) -> usize {
        // uart_reg.h: STATUS считает FIFO без регистра сдвига.
        self.tx.len().saturating_sub(1)
    }

    fn raw(&self) -> u32 {
        // uart_ll_set_rxfifo_full_thr / uart_ll_set_txfifo_empty_thr: строгие сравнения.
        let rx_full = self.rx.len() as u32 > self.active.thresholds & THRESHOLD_MASK;
        let tx_empty =
            (self.tx_fifo_count() as u32) < (self.active.thresholds >> 9) & THRESHOLD_MASK;
        self.latched | u32::from(rx_full) * RX_FULL | u32::from(tx_empty) * TX_EMPTY
    }

    pub fn irq(&self) -> bool {
        self.raw() & self.enable != 0
    }

    pub fn take_tx(&mut self) -> Vec<(u64, u8)> {
        std::mem::take(&mut self.done)
    }

    fn clear_rx_fifo(&mut self) {
        self.rx.clear();
        self.rx_deadline = None;
    }

    /// Доступ 32 бит по смещению от UART0/1, включая UART_FIFO_AHB_REG.
    /// Базы 0x60000000 и 0x60010000 описаны в esp32c3/include/soc/soc.h.
    pub fn access(
        &mut self,
        offset: u32,
        write: Option<u32>,
        now: u64,
    ) -> Result<Option<u32>, BoardFault> {
        self.tick(now)?;
        // Смещения и маски ниже соответствуют одноимённым полям uart_reg.h.
        let Some(value) = write else {
            return Ok(Some(match offset {
                0x00 => {
                    let byte = self
                        .rx
                        .pop_front()
                        .ok_or_else(|| invalid("UART: RX FIFO пуст"))?;
                    if self.rx.is_empty() {
                        self.rx_deadline = None;
                    }
                    u32::from(byte)
                }
                0x04 => self.raw(),               // UART_INT_RAW
                0x08 => self.raw() & self.enable, // UART_INT_ST
                0x0c => self.enable,              // UART_INT_ENA
                0x14 => self.shadow.divider,      // UART_CLKDIV
                0x18 => self.filter,              // UART_RX_FILT
                0x1c => (self.tx_fifo_count() as u32) << 16 | self.rx.len() as u32, // UART_STATUS: счётчики.
                0x20 => self.shadow.format,                                         // UART_CONF0
                0x24 => self.shadow.thresholds,                                     // UART_CONF1
                0x48 => self.idle,                           // UART_IDLE_CONF
                0x60 => self.shadow.memory,                  // UART_MEM_CONF
                0x6c => u32::from(!self.tx.is_empty()) << 4, // UART_FSM_STATUS: форма занятости для uart_ll_is_tx_idle.
                0x78 => self.shadow.clock,                   // UART_CLK_CONF
                0x7c => 0x0200_8270,                         // UART_DATE reset.
                0x80 => self.id,
                _ => return Ok(None),
            }));
        };

        let allowed = |mask: u32| {
            if value & !mask == 0 {
                Ok(())
            } else {
                Err(unsupported(&format!(
                    "UART: неподдерживаемые поля регистра {offset:#x}"
                )))
            }
        };
        let idle = || {
            if !self.wire.is_empty() || !self.tx.is_empty() {
                Err(unsupported(
                    "UART: смена формата или частоты во время кадра",
                ))
            } else {
                Ok(())
            }
        };

        match offset {
            0x00 => {
                if value > u32::from(u8::MAX) || self.tx_fifo_count() == FIFO_LEN {
                    return Err(invalid("UART: переполнение TX FIFO или неверный байт"));
                }
                let (half_bit, scale) = self.half_bit(true)?;
                if self.idle >> 10 != 0 {
                    return Err(unsupported("UART: ненулевой TX_IDLE_NUM"));
                }
                let start = self
                    .tx
                    .back()
                    .map_or(self.now * scale, |frame| frame.phase_end);
                let end = start + self.frame_half_bits()? * half_bit;
                if end.div_ceil(scale) > u128::from(u64::MAX) {
                    return Err(invalid("UART: конец кадра вне шкалы времени"));
                }
                self.tx.push_back(Frame {
                    start: start.div_ceil(scale),
                    end: end.div_ceil(scale),
                    phase_end: end,
                    timeout: None,
                    byte: value as u8,
                });
            }
            0x0c => {
                allowed(DRIVER_IRQ)?;
                self.enable = value;
            }
            0x10 => self.latched &= !value, // UART_INT_CLR, W1C.
            0x14 => {
                allowed(0x00f0_0fff)?; // UART_CLKDIV/CLKDIV_FRAG.
                idle()?;
                self.shadow.divider = value;
            }
            0x18 => {
                allowed(0xff)?; // UART_GLITCH_FILT; включение требует входа по фронтам.
                self.filter = value;
            }
            0x20 => {
                allowed(MEM_CLK_EN | (1 << 25) | (1 << 26) | RX_FIFO_RESET | TX_FIFO_RESET | 0xff)?;
                if (value ^ self.shadow.format) & !(RX_FIFO_RESET | TX_FIFO_RESET) != 0 {
                    idle()?;
                }
                if value & RX_FIFO_RESET != 0 {
                    self.clear_rx_fifo();
                }
                if value & TX_FIFO_RESET != 0 {
                    self.tx.truncate(1); // UART_TXFIFO_RST очищает очередь FIFO.
                }
                self.shadow.format = value;
            }
            0x24 => {
                allowed(RX_TIMEOUT_EN | 0x3ffff)?; // UART_TXFIFO_EMPTY_THRHD и RXFIFO_FULL_THRHD.
                if (self.shadow.thresholds ^ value) & RX_TIMEOUT_EN != 0 && !self.wire.is_empty() {
                    return Err(unsupported("UART: изменение тайм-аута во время кадра"));
                }
                self.shadow.thresholds = value;
            }
            0x48 => {
                allowed(0xfffff)?;
                self.idle = value;
            }
            0x60 => {
                allowed(0x0fff_fffe)?;
                if value & 0x7e != 0x12 {
                    // UART_{RX,TX}_SIZE, банк по 128 байт.
                    return Err(unsupported("UART: перераспределение RAM между FIFO"));
                }
                if self.shadow.memory != value {
                    idle()?;
                }
                self.shadow.memory = value;
            }
            0x78 => {
                allowed(0x0fff_ffff)?;
                if (value ^ self.shadow.clock) & !(RX_RESET | TX_RESET | CORE_RESET) != 0 {
                    idle()?;
                }
                if value & (RX_RESET | CORE_RESET) != 0 {
                    self.clear_rx_fifo();
                    self.wire.clear();
                    self.latched &= !(RX_OVERFLOW | RX_TIMEOUT);
                }
                if value & (TX_RESET | CORE_RESET) != 0 {
                    self.tx.clear();
                    self.latched &= !TX_DONE;
                }
                self.shadow.clock = value;
            }
            0x80 => {
                if value & (HIGH_SPEED | UPDATE) != 0 && self.shadow != self.active {
                    idle()?;
                }
                self.id = value & !UPDATE; // UART_UPDATE завершается на границе MMIO.
            }
            _ => return Ok(None),
        }
        if self.id & HIGH_SPEED != 0 || offset == 0x80 && value & UPDATE != 0 {
            if self.active.thresholds & RX_TIMEOUT_EN != self.shadow.thresholds & RX_TIMEOUT_EN
                || self.active.memory != self.shadow.memory
            {
                self.rx_deadline = None;
            }
            self.active = self.shadow;
        }
        Ok(Some(0))
    }
}
