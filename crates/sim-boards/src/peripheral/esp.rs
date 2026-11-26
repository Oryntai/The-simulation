//! ESP32-C3 TRM: IO MUX/GPIO, Timer Group, MWDT, UART,
//! LEDC и ADC. Поля GPIO, LEDC и UART сверены с ESP-IDF v6.1.1.

use super::esp_assist::EspAssist;
use super::esp_bus::EspBuses;
use super::esp_dma::EspDma;
use super::esp_pms::EspPms;
use super::esp_rtc::EspRtc;
use super::esp_sar_digital::{self, EspSarDigital};
use super::esp_sha::EspSha;
use super::esp_sleep::{GPIO_HIGH, GPIO_LOW, RTC_GPIO_COUNT};
use super::esp_sleep_regs::EspSleepRegisters;
use super::esp_timg::{self, EspTimg, MwdtReset};
use super::esp_uart::EspUart;
use super::{
    esp_flash::EspFlash,
    esp_system::{EspSystem, SystimerPhase},
};
use crate::esp_clock::{EspClockRates, EspClockSource, QUARTERS_PER_SECOND, XTAL_HZ};
use crate::esp_memory::EspMemory;
use crate::{BoardFault, EspSleepMode, GpioMode, InputOutcome, PwmOutput};
use std::collections::VecDeque;

use super::{Access, Adc, Pins, Uart, checked_rw, command, invalid, ro, rw, unsupported};

const GPIO_MASK: u32 = (1 << 22) - 1;
// TRM рег. 5.3, 5.4, 5.6, 5.7, 5.12: WT задаёт эффект записи, чтение не описано.
// gpio_ll.h задаёт эти поля через RMW. Для такого чтения нужен явный профиль.
pub(super) fn gpio_write_trigger_field(address: u32) -> Option<&'static str> {
    match address {
        0x6000_4008 => Some("GPIO_OUT_W1TS"),
        0x6000_400c => Some("GPIO_OUT_W1TC"),
        0x6000_4024 => Some("GPIO_ENABLE_W1TS"),
        0x6000_4028 => Some("GPIO_ENABLE_W1TC"),
        0x6000_404c => Some("GPIO_STATUS_W1TC"),
        _ => None,
    }
}

pub(super) fn supports_reset_value(address: u32, field: &str, value: u32) -> bool {
    gpio_write_trigger_field(address) == Some(field) && value & !GPIO_MASK == 0
        || (address, field) == super::esp_system::WFI_STALL_PROFILE && value <= 1
}

const TIMER_MASK: u64 = (1 << 54) - 1;
const WDT_KEY: u32 = 0x50d8_3aa1;

// ESP-IDF soc/esp32c3/register/soc/system_reg.h, SYSTEM_PERIP_{CLK,RST}_EN0.
const PERIP_CLK_EN0: u32 = crate::esp_clock::SYSTEM + 0x10;
const PERIP_CLK_EN1: u32 = crate::esp_clock::SYSTEM + 0x14;
const PERIP_RST_EN1: u32 = crate::esp_clock::SYSTEM + 0x1c;
const GDMA_GATE: u32 = 1 << 6;
const SHA_GATE: u32 = 1 << 2; // SYSTEM_CRYPTO_SHA_{CLK_EN,RST}.
// system_reg.h, SYSTEM_CRYPTO_{DS,HMAC}_RST, и esp32c3/hal/sha_ll.h:
// SHA удерживается в сбросе, если активен любой из этих двух сбросов.
const SHA_DEPENDENCY_RESETS: u32 = (1 << 4) | (1 << 5);
const PERIP_RST_EN1_RESET: u32 = 0x1fe; // SYSTEM_PERIP_RST_EN1: биты 1…8 сброшены в 1.
const PERIP_RST_EN0: u32 = crate::esp_clock::SYSTEM + 0x18;
const PERIP_CLK_EN0_RESET: u32 = 0xf9c1_e06f;
const PERIP_EN0_RW: u32 = u32::MAX & !(1 << 4); // system_reg.h: бит 4 зарезервирован.
const PERIP_EN1_RW: u32 = 0x1fe | (1 << 10); // system_reg.h: биты 1…8 и TSENS.
const SPI01_GATE: u32 = 1 << 1;
const UART_GATE: [u32; 2] = [1 << 2, 1 << 5];
const UART_MEM: u32 = 1 << 24;
const I2C_GATE: u32 = 1 << 7;
const LEDC_GATE: u32 = 1 << 11;
const SPI_GATE: u32 = 1 << 6;
const SPI_DMA_GATE: u32 = 1 << 22;
// gpio_sig_map.h: индексы GPIO Matrix.
const UART_DATA: [u32; 2] = [6, 9];
const I2C_LINES: [u32; 2] = [53, 54];
const SPI_CLOCK: u32 = 63;
const SPI_MISO: u32 = 64;
const SPI_MOSI: u32 = 65;
const SPI_CS0: u32 = 68;
const LEDC_FIRST: u32 = 45;
const GPIO_OUT: u32 = 128;
const LOOPBACK: std::ops::RangeInclusive<u32> = 97..=100;
// gpio_reg.h: FUNCx_IN_SEL_CFG и FUNCx_OUT_SEL_CFG.
const MATRIX_IN: u32 = 1 << 6;
const INPUT_INVERT: u32 = 1 << 5;
const INPUT_PAD: u32 = 0x1f;
const INPUT_ONE: u32 = 0x1e;
const INPUT_ZERO: u32 = 0x1f;
const OUTPUT_SIGNAL: u32 = 0xff;
const OUTPUT_INVERT: u32 = 1 << 8;
const ENABLE_FROM_GPIO: u32 = 1 << 9;
const ENABLE_INVERT: u32 = 1 << 10;
const PAD_OPEN_DRAIN: u32 = 1 << 2;
// TRM регистр GPIO_PINn_REG и gpio_reg.h: пробуждение цифрового GPIO.
const PIN_WAKE_ENABLE: u32 = 1 << 10;
const PIN_INT_SHIFT: u32 = 7;
const PIN_INT_TYPE: u32 = 0b111 << PIN_INT_SHIFT;
// io_mux_reg.h: поля IO_MUX_GPIOn.
const INPUT_ENABLE: u32 = 1 << 9;
const GPIO_FUNC: u32 = 1;
const MUX_FUNC_SHIFT: u32 = 12;
const PULL_UP: u32 = 1 << 8;
const PULL_DOWN: u32 = 1 << 7;
const PAD_ACTIVE_MASK: u32 = 0x7fff;
// io_mux_reg.h: FUNC_SPI{HD,WP,CS0,CLK,D,Q}_SPI* — прямая шина SPI0/1.
const FLASH_PADS: std::ops::RangeInclusive<usize> = 12..=17;
const SLEEP_OUTPUT_ENABLE: u32 = 1;
const SLEEP_SELECT: u32 = 1 << 1;
const SLEEP_PULL_DOWN: u32 = 1 << 2;
const SLEEP_PULL_UP: u32 = 1 << 3;
const SLEEP_INPUT_ENABLE: u32 = 1 << 4;
const PAD_SLEEP_FIELDS: u32 = 0x7f; // io_mux_reg.h: SLP_DRV[6:5], SLP_IE…SLP_OE[4:0].
// ledc_reg.h: LSTIMERx_CONF, LSCHx_CONF0/1.
const LED_TIMER_RESET: u32 = 1 << 23;
const LED_TIMER_PAUSE: u32 = 1 << 22;
const LED_TIMER_UPDATE: u32 = 1 << 25;
const LED_TIMER_TICK_SEL: u32 = 1 << 24;
const LED_DUTY_START: u32 = 1 << 31;
const LED_DUTY_INC: u32 = 1 << 30;
const LED_OUTPUT_ENABLE: u32 = 1 << 2;
const LED_IDLE: u32 = 1 << 3;
const LED_CHANNEL_UPDATE: u32 = 1 << 4;
// TRM 6.2: прежний ELF-профиль использует PLL, APB=CPU=80 МГц.
const PLL80_HZ: u32 = 80_000_000;
// ESP-IDF soc/esp32c3/register/soc/reg_base.h, DR_REG_SYSTIMER_BASE и SPI2.
const SYSTIMER_WINDOW: std::ops::Range<u32> = 0x6002_3000..0x6002_4000;
// ESP-IDF usb_serial_jtag_reg.h, TRM §30.3.2 и рег. 30.11.
const USB_EP1_FIFO: u32 = 0x6004_3000;
const USB_EP1_CONF: u32 = USB_EP1_FIFO + 4;
const USB_INT_RAW: u32 = USB_EP1_FIFO + 0x08;
const USB_INT_CLR: u32 = USB_EP1_FIFO + 0x14;
const USB_SOF_INT: u32 = 1 << 1;
const USB_SERIAL_IN_EMPTY: u32 = 1 << 3;
const USB_WR_DONE: u32 = 1 << 0;
const USB_IN_FREE: u32 = 1 << 1;
const USB_CDC_PACKET: u8 = 64;

#[derive(Clone)]
struct Group {
    cfg: u32,
    counter: u64,
    latched: u64,
    load: [u32; 2],
    alarm: [u32; 2],
    phase: u64,
    int_en: u32,
    raw: u32,
    wdt_key: u32,
    wdt_setup: u32,
    wdt_div: u32,
    wdt_hold: u32,
    deadline: Option<u64>,
}

impl Default for Group {
    fn default() -> Self {
        Self {
            cfg: 0x6000_2000,
            counter: 0,
            latched: 0,
            load: [0; 2],
            alarm: [0; 2],
            phase: 0,
            int_en: 0,
            raw: 0,
            wdt_key: WDT_KEY,
            wdt_setup: 0x0004_8000,
            wdt_div: 1 << 16,
            wdt_hold: 1_048_575,
            deadline: None,
        }
    }
}

impl Group {
    fn tick(&mut self, cycles: u64, now: u64) -> bool {
        if self.cfg >> 31 != 0 {
            let div = match self.cfg >> 13 & 0xffff {
                0 => 65536,
                1 | 2 => 2,
                n => n,
            };
            let div = u64::from(div) * if self.cfg & (1 << 9) != 0 { 2 } else { 1 };
            self.phase += cycles;
            let ticks = self.phase / div;
            self.phase %= div;
            if ticks != 0 {
                let up = self.cfg & 1 << 30 != 0;
                let target = pair(self.alarm);
                let diff = if up {
                    target.wrapping_sub(self.counter)
                } else {
                    self.counter.wrapping_sub(target)
                } & TIMER_MASK;
                if self.cfg & (1 << 10) != 0 && diff != 0 && diff <= ticks {
                    self.raw |= 1;
                    self.cfg &= !(1 << 10);
                    if self.cfg & 1 << 29 != 0 {
                        self.counter = pair(self.load);
                    } else {
                        self.counter = target;
                    }
                    self.advance(ticks - diff, up);
                } else {
                    self.advance(ticks, up);
                }
            }
        }
        self.deadline.is_some_and(|d| now >= d)
    }

    fn advance(&mut self, ticks: u64, up: bool) {
        self.counter = if up {
            self.counter.wrapping_add(ticks)
        } else {
            self.counter.wrapping_sub(ticks)
        } & TIMER_MASK;
    }

    fn feed(&mut self, now: u64) -> Result<(), crate::BoardFault> {
        if self.wdt_setup >> 31 == 0 {
            self.deadline = None;
            return Ok(());
        }
        let div = self.wdt_div >> 16;
        if div == 0 || self.wdt_hold == 0 {
            return Err(invalid("нулевой делитель или тайм-аут MWDT"));
        }
        let mul = if self.wdt_setup & 1 << 21 != 0 { 2 } else { 1 };
        let delay = u64::from(self.wdt_hold) * u64::from(div) * mul;
        self.deadline = Some(now + delay);
        Ok(())
    }

    fn access(&mut self, o: u32, v: Option<u32>, now: u64) -> Access {
        if (0x48..=0x60).contains(&o) && v.is_some() && self.wdt_key != WDT_KEY {
            return Err(invalid("MWDT заблокирован для записи"));
        }
        match o {
            0 => {
                if v.is_some_and(|v| v & 1 << 12 != 0) {
                    self.phase = 0;
                }
                checked_rw(
                    v.map(|v| v & !(1 << 12)),
                    &mut self.cfg,
                    0xffff_e600,
                    "TIMG T0CONFIG",
                )
            }
            4 => ro(v, self.latched as u32),
            8 => ro(v, (self.latched >> 32) as u32),
            0xc => {
                if v.is_some() {
                    self.latched = self.counter;
                    Ok(Some(0))
                } else {
                    ro(v, 0)
                }
            }
            0x10 | 0x14 => rw(
                v,
                &mut self.alarm[((o - 0x10) / 4) as usize],
                if o == 0x10 { u32::MAX } else { 0x3f_ffff },
            ),
            0x18 | 0x1c => rw(
                v,
                &mut self.load[((o - 0x18) / 4) as usize],
                if o == 0x18 { u32::MAX } else { 0x3f_ffff },
            ),
            0x20 => command(v, |_| self.counter = pair(self.load)),
            0x48 => {
                if let Some(v) = v {
                    if v & !0xe07f_8000 != 0 || v >> 31 != 0 && v >> 29 & 3 != 3 {
                        return Err(unsupported("MWDT: только сброс системы на первой стадии"));
                    }
                    if v >> 31 != 0 && v & 1 << 22 == 0 {
                        return Err(invalid("MWDT требует WDT_CONF_UPDATE_EN"));
                    }
                    self.wdt_setup = v & !(1 << 22);
                    self.feed(now)?;
                    Ok(Some(0))
                } else {
                    ro(v, self.wdt_setup)
                }
            }
            0x4c | 0x50 if self.deadline.is_some() && v.is_some() => Err(unsupported(
                "изменение делителя или порога работающего MWDT",
            )),
            0x4c => rw(v, &mut self.wdt_div, 0xffff_0000),
            0x50 => rw(v, &mut self.wdt_hold, u32::MAX),
            0x60 if v.is_some() => {
                self.feed(now)?;
                Ok(Some(0))
            }
            0x64 => rw(v, &mut self.wdt_key, u32::MAX),
            0x70 => checked_rw(v, &mut self.int_en, 1, "TIMG INT_ENA"),
            0x74 => ro(v, self.raw),
            0x78 => ro(v, self.raw & self.int_en),
            0x7c => command(v, |v| self.raw &= !v),
            _ => Ok(None),
        }
    }
}

fn pair(parts: [u32; 2]) -> u64 {
    u64::from(parts[0]) | (u64::from(parts[1]) << 32)
}

#[derive(Clone)]
struct LedChannel {
    cfg: u32,
    shadow: u32,
    hpoint: u32,
    active_hpoint: u32,
    duty: u32,
    duty_cfg: u32,
    active: u32,
    start: bool,
    pending: Option<(u32, u32, u32)>,
}

impl Default for LedChannel {
    fn default() -> Self {
        Self {
            cfg: 0,
            shadow: 0,
            hpoint: 0,
            active_hpoint: 0,
            duty: 0,
            duty_cfg: LED_DUTY_INC, // ledc_reg.h: CONF1 reset.
            active: 0,
            start: false,
            pending: None,
        }
    }
}
#[derive(Clone)]
struct LedTimer {
    shadow: u32,
    cfg: u32,
    ctr: u32,
    phase: u64,
    pending: Option<u32>,
}

impl Default for LedTimer {
    fn default() -> Self {
        Self {
            shadow: LED_TIMER_RESET,
            cfg: LED_TIMER_RESET,
            ctr: 0,
            phase: 0,
            pending: None,
        }
    }
}

impl LedTimer {
    fn params(&self) -> Option<(u32, u128)> {
        let div = u128::from(self.cfg >> 4 & 0x3ffff);
        let bits = self.cfg & 15;
        (div != 0 && bits != 0).then(|| (1 << bits, div))
    }

    fn tick(&mut self, source_ticks: u128) -> bool {
        let Some((period, div)) = self.params() else {
            return false;
        };
        if self.cfg & (LED_TIMER_RESET | LED_TIMER_PAUSE) != 0 {
            return false;
        }
        // TRM §32.3.2: восемь дробных бит делителя и применение новой
        // настройки при первом переполнении старого счётчика.
        let phase = u128::from(self.phase) + source_ticks * 256;
        let count = u128::from(self.ctr) + phase / div;
        let overflow = count >= u128::from(period);
        if let Some(next) = self.pending
            && overflow
        {
            let until_wrap =
                (u128::from(period - self.ctr) * div - u128::from(self.phase)).div_ceil(256);
            self.cfg = next;
            self.pending = None;
            self.ctr = 0;
            self.phase = 0;
            self.tick(source_ticks - until_wrap);
        } else {
            self.ctr = (count % u128::from(period)) as u32;
            self.phase = (phase % div) as u64;
        }
        overflow
    }
}

// TRM UART: аппаратный FIFO 128 байт; входная очередь хоста на столько же кадров.
pub(super) const UART_FIFO: usize = 128;
const RX_FULL: u32 = 1;
const RX_OVERFLOW: u32 = 1 << 4;

#[derive(Clone)]
struct Serial {
    div: u32,
    config: u32,
    clock: u32,
    wire: VecDeque<(u64, u8)>,
    wire_end_x16: u64,
    rx: VecDeque<u8>,
    threshold: u32,
    overflow: bool,
    enable: u32,
}
impl Default for Serial {
    fn default() -> Self {
        Self {
            div: 0x2b6,
            config: 0x1000_001c,
            clock: 0x0370_1000,
            wire: VecDeque::new(),
            wire_end_x16: 0,
            rx: VecDeque::new(),
            threshold: 96, // UART_CONF1.RXFIFO_FULL_THRHD, reset.
            overflow: false,
            enable: 0,
        }
    }
}

impl Serial {
    fn frame_cycles_x16(&self, transmit: bool) -> Result<u64, BoardFault> {
        if self.config & 0x1000_003f != 0x1000_001c {
            return Err(invalid("UART требует 8N1 и включённую память"));
        }
        let gate = if transmit { 1 << 24 } else { 1 << 25 };
        let gate_mask = gate | (1 << 22);
        let multiplier = match (self.clock >> 20) & 3 {
            1 => 1, // PLL80 в профиле работает на частоте CPU.
            3 => 2, // XTAL 40 МГц.
            _ => return Err(unsupported("источник тактов UART")),
        };
        if self.clock & gate_mask != gate_mask || self.clock & 0xfff != 0 {
            return Err(unsupported("делитель/тактирование UART"));
        }
        let div16 = u64::from((self.div & 0xfff) * 16 + (self.div >> 20 & 15));
        let div = u64::from((self.clock >> 12 & 255) + 1);
        if div16 == 0 {
            return Err(invalid("нулевой делитель UART"));
        }
        // Один стартовый, восемь данных, один стоповый.
        Ok(10 * div16 * div * multiplier)
    }

    fn receive(&mut self, now: u64, bytes: &[u8]) -> Result<InputOutcome, BoardFault> {
        let duration = self.frame_cycles_x16(false)?;
        let accepted = bytes.len().min(UART_FIFO - self.wire.len());
        let mut at = self.wire_end_x16.max(now * 16);
        for &byte in &bytes[..accepted] {
            at += duration;
            self.wire.push_back((at.div_ceil(16), byte));
        }
        self.wire_end_x16 = at;
        Ok(InputOutcome::Uart {
            accepted,
            dropped: bytes.len() - accepted,
        })
    }

    fn tick(&mut self, now: u64) {
        while let Some((_, byte)) = self.wire.pop_front_if(|(at, _)| *at <= now) {
            if self.rx.len() == UART_FIFO {
                self.overflow = true;
            } else {
                self.rx.push_back(byte);
            }
        }
    }

    fn raw(&self) -> u32 {
        (u32::from(self.rx.len() as u32 > self.threshold) * RX_FULL)
            | (u32::from(self.overflow) * RX_OVERFLOW)
    }
}

#[derive(Clone)]
pub(crate) struct BootPeripherals {
    pub(crate) reset_values: crate::esp_reset_profile::EspResetValues,
    pub(crate) virtual_rng: Option<crate::esp_rng::EspVirtualRng>,
    pub(super) timg: [EspTimg; esp_timg::BASES.len()],
    sar: EspSarDigital,
    sleep_regs: EspSleepRegisters,
    pub(crate) system: EspSystem,
    pub(crate) flash: EspFlash,
    pub(crate) pms: EspPms,
    pub(crate) wdt_delay_sel: u8,
    pub(crate) pc: u32,
    clk_io: u32,
    rst_io: u32,
    clk_en1: u32,
    rst_en1: u32,
    sha: EspSha,
    has_systimer_phase: bool,
    usb_ep1_bytes: u8,
    assist: EspAssist,
    systimer_wfi_stall: Option<bool>,
}

impl BootPeripherals {
    pub(super) fn new(
        flash: EspFlash,
        phase: Option<SystimerPhase>,
        wdt_delay_sel: u8,
        profile: Option<crate::EspResetProfile>,
    ) -> Self {
        Self {
            reset_values: crate::esp_reset_profile::EspResetValues::new(profile),
            virtual_rng: None,
            timg: std::array::from_fn(|_| EspTimg::default()),
            sar: EspSarDigital::default(),
            sleep_regs: EspSleepRegisters::default(),
            // TRM §10.3 задаёт /2 и /3. Без входной фазы доступна только
            // шкала CPU и общие поля SYSREG, не состояние SYSTIMER.
            system: EspSystem::new(phase.unwrap_or(SystimerPhase::FirstAfterTwoXtalCycles)),
            has_systimer_phase: phase.is_some(),
            flash,
            pms: EspPms::default(),
            wdt_delay_sel,
            pc: 0,
            clk_io: PERIP_CLK_EN0_RESET & !EspSystem::shared_sysreg_mask(PERIP_CLK_EN0),
            rst_io: 0,
            clk_en1: 0,
            rst_en1: PERIP_RST_EN1_RESET,
            sha: EspSha::default(),
            usb_ep1_bytes: 0,
            assist: EspAssist::default(),
            systimer_wfi_stall: None,
        }
    }

    pub(super) fn reset(&mut self) {
        self.flash.reset();
        self.pms = EspPms::default();
        self.system.reset();
        self.clk_io = PERIP_CLK_EN0_RESET & !EspSystem::shared_sysreg_mask(PERIP_CLK_EN0);
        self.rst_io = 0;
        self.clk_en1 = 0;
        self.rst_en1 = PERIP_RST_EN1_RESET;
        self.sha = EspSha::default();
        self.usb_ep1_bytes = 0;
        self.assist = EspAssist::default();
        self.timg = std::array::from_fn(|_| EspTimg::default());
        self.sar = EspSarDigital::default();
        self.sleep_regs = EspSleepRegisters::default();
    }

    pub(crate) fn assist_interrupt(&self) -> bool {
        self.assist.interrupt()
    }

    pub(crate) fn observe_cpu(&mut self, pc: u32, sp: u32) {
        self.assist.observe(pc, sp);
    }

    pub(crate) fn systimer_wfi_stall(&mut self) -> Result<bool, BoardFault> {
        if let Some(stall) = self.systimer_wfi_stall {
            return Ok(stall);
        }
        let (address, field) = super::esp_system::WFI_STALL_PROFILE;
        // TRM §10.4 и рег. 16.8 не связывают CPU stall с затвором WFI.
        // Сохраняем прежнюю модель без явного профиля. QEMU считает
        // QEMU_CLOCK_VIRTUAL независимо от затвора (hw/timer/esp_systimer.c).
        let stall = self.reset_values.read(address, field)? != Some(0);
        self.systimer_wfi_stall = Some(stall);
        Ok(stall)
    }

    pub(crate) fn systimer_interrupt_status(&self) -> Option<u32> {
        self.has_systimer_phase
            .then(|| self.system.interrupt_status())
    }

    pub(super) fn access(
        &mut self,
        chip: &mut Esp,
        rtc: Option<&EspRtc>,
        now: u64,
        addr: u32,
        size: u8,
        value: Option<u32>,
    ) -> Access {
        if value.is_none()
            && size as usize == size_of::<u32>()
            && let Some(field) = gpio_write_trigger_field(addr)
        {
            return self.reset_values.read(addr, field)?.map(Some).ok_or(
                BoardFault::UnsupportedRegister {
                    pc: self.pc,
                    addr,
                    size,
                    write: false,
                    block: "GPIO: чтение WT не задокументировано",
                },
            );
        }
        if let Some(word) =
            self.sleep_regs
                .access(self.pc, addr, size, value, &mut self.reset_values)?
        {
            return Ok(Some(word));
        }
        if addr == crate::esp_rng::ADDRESS {
            if size as usize != size_of::<u32>() || value.is_some() {
                return Err(BoardFault::UnsupportedRegister {
                    pc: self.pc,
                    addr,
                    size,
                    write: value.is_some(),
                    block: "RNG_DATA_REG",
                });
            }
            return self
                .virtual_rng
                .as_mut()
                .ok_or_else(|| BoardFault::Unsupported("RNG: источник энтропии не задан".into()))?
                .read()
                .map(Some);
        }
        if addr & !0xfff == esp_sar_digital::BASE {
            // bootloader_random_esp32c3.c: после закрытия APB_SARADC_CLK_EN
            // SDK снимает TIMER_EN и очищает таблицу. Доступна конфигурация
            // выключенного автомата, а результаты преобразования не доступны.
            return self.sar.access(
                self.pc,
                addr,
                size,
                value,
                self.rst_io & esp_sar_digital::ADC_GATE == 0,
            );
        }
        if let Some(group) = esp_timg::BASES
            .iter()
            .position(|&base| addr & !0xfff == base)
        {
            let enabled = self.enabled(esp_timg::GATES[group]);
            return self.timg[group].access(
                self.pc,
                addr,
                addr - esp_timg::BASES[group],
                size,
                value,
                enabled,
            );
        }
        if let Some(result) = self.assist.access(self.pc, addr, size, value)? {
            return Ok(Some(result));
        }
        if let Some(result) = self.pms.access(self.pc, addr, size, value)? {
            return Ok(Some(result));
        }
        let sha_enabled =
            self.clk_en1 & SHA_GATE != 0 && self.rst_en1 & (SHA_GATE | SHA_DEPENDENCY_RESETS) == 0;
        if let Some(result) = self.sha.access(self.pc, addr, size, value, sha_enabled)? {
            return Ok(Some(result));
        }
        if matches!(
            addr,
            USB_EP1_FIFO | USB_EP1_CONF | USB_INT_RAW | USB_INT_CLR
        ) {
            return self.usb_ep1_access(addr, size, value);
        }
        if !self.has_systimer_phase && SYSTIMER_WINDOW.contains(&addr) {
            return Err(BoardFault::UnsupportedRegister {
                pc: self.pc,
                addr,
                size,
                write: value.is_some(),
                block: "SYSTIMER: не задана начальная фаза",
            });
        }
        if [PERIP_CLK_EN1, PERIP_RST_EN1].contains(&addr) {
            return self.access_accelerator_gates(chip, now, addr, size, value);
        }
        let mask = EspSystem::shared_sysreg_mask(addr);
        if mask == 0 {
            // Пакетные запросы задают срок через публичные такты CPU.
            // Нельзя менять длительность такта до завершения уже выданного запроса.
            let bus_active = chip.buses.active();
            if value.is_some()
                && [
                    crate::esp_clock::SYSCLK_CONF,
                    crate::esp_clock::CPU_PER_CONF,
                ]
                .contains(&addr)
            {
                let mut next = self.system.clone();
                let result = next.access(self.pc, addr, size, value)?;
                let clocks = next.clocks();
                if bus_active && clocks.cpu_hz() != self.system.clocks().cpu_hz() {
                    return Err(invalid("смена частоты CPU во время обмена I²C/SPI"));
                }
                if clocks.source() == EspClockSource::Pll
                    && let Some(rtc) = rtc
                {
                    rtc.check_pll_selection(clocks.pll_selected_hz())?;
                }
                self.system = next;
                return Ok(result);
            }
            return self.system.access(self.pc, addr, size, value);
        }

        let previous = match addr {
            PERIP_CLK_EN0 => self.clk_io,
            PERIP_RST_EN0 => self.rst_io,
            _ => unreachable!("известные общие регистры ESP32-C3"),
        };
        let unknown = || BoardFault::UnsupportedRegister {
            pc: self.pc,
            addr,
            size,
            write: value.is_some(),
            block: "поля SYSREG других блоков",
        };
        if value.is_some_and(|bits| bits & !PERIP_EN0_RW != 0) {
            return Err(unknown());
        }
        if addr == PERIP_CLK_EN0
            && let Some(bits) = value
        {
            let disabled = previous & !bits;
            if (disabled & I2C_GATE != 0 && chip.buses.i2c_active())
                || (disabled & SPI_GATE != 0 && chip.buses.spi_active())
                || (disabled & SPI_DMA_GATE != 0 && chip.buses.spi_dma_active())
            {
                return Err(unsupported("закрытие тактов во время обмена I²C/SPI"));
            }
        }
        let result = self
            .system
            .access(self.pc, addr, size, value.map(|bits| bits & mask))?;
        if let Some(bits) = value {
            // esp_system/port/soc/esp32c3/clk.c использует общее слово.
            // Доступ к регистрам каждого неподдержанного блока остаётся ошибкой.
            let io = bits & !mask;
            if addr == PERIP_RST_EN0 {
                let asserted = io & !previous;
                if asserted & esp_sar_digital::ADC_GATE != 0 {
                    self.sar = EspSarDigital::default();
                }
                for (group, gate) in esp_timg::GATES.into_iter().enumerate() {
                    if asserted & gate != 0 {
                        self.timg[group] = EspTimg::default();
                    }
                }
                for (port, bit) in UART_GATE.into_iter().enumerate() {
                    if asserted & (bit | UART_MEM) != 0 {
                        chip.sdk_uart[port] = sdk_uart();
                        chip.uart_source[port] = 3;
                        chip.uart_time[port] = 0;
                    }
                }
                if asserted & LEDC_GATE != 0 {
                    chip.led_clock = 0;
                    chip.led_timers = std::array::from_fn(|_| LedTimer::default());
                    chip.channels = std::array::from_fn(|_| LedChannel::default());
                }
                chip.buses.reset_io(
                    asserted & I2C_GATE != 0,
                    asserted & (SPI_GATE | SPI_DMA_GATE) != 0,
                    now,
                );
                self.rst_io = io;
            } else {
                self.clk_io = io;
            }
            self.flash
                .set_spi01_control(self.clk_io & SPI01_GATE != 0, self.rst_io & SPI01_GATE != 0);
        }
        Ok(result.map(|bits| {
            if value.is_none() {
                bits | previous
            } else {
                bits
            }
        }))
    }

    fn usb_ep1_access(&mut self, addr: u32, size: u8, value: Option<u32>) -> Access {
        // TRM §30.3.2: без USB-хоста OUT пуст, а отправленный пакет IN не
        // освобождает буфер. Пакет готов после WR_DONE или заполнения 64 байт.
        if size as usize == size_of::<u32>() {
            match (addr, value) {
                // usb_serial_jtag_reg.h: IN_EMPTY защёлкнут при сбросе.
                // Без хоста нет SOF и пакетов, а очистка SOF не меняет IN_EMPTY.
                (USB_INT_RAW, None) => return Ok(Some(USB_SERIAL_IN_EMPTY)),
                (USB_INT_CLR, Some(USB_SOF_INT)) => return Ok(Some(0)),
                (USB_EP1_CONF, None) => {
                    return Ok(Some(
                        u32::from(self.usb_ep1_bytes < USB_CDC_PACKET) * USB_IN_FREE,
                    ));
                }
                (USB_EP1_CONF, Some(bits)) if bits & !(USB_WR_DONE | USB_IN_FREE) == 0 => {
                    if bits & USB_WR_DONE != 0 {
                        self.usb_ep1_bytes = USB_CDC_PACKET;
                    }
                    return Ok(Some(0));
                }
                (USB_EP1_FIFO, Some(byte))
                    if byte <= u32::from(u8::MAX) && self.usb_ep1_bytes < USB_CDC_PACKET =>
                {
                    self.usb_ep1_bytes += 1;
                    return Ok(Some(0));
                }
                _ => {}
            }
        }
        Err(BoardFault::UnsupportedRegister {
            pc: self.pc,
            addr,
            size,
            write: value.is_some(),
            block: "USB Serial/JTAG EP1 без хоста",
        })
    }

    fn access_accelerator_gates(
        &mut self,
        chip: &mut Esp,
        now: u64,
        addr: u32,
        size: u8,
        value: Option<u32>,
    ) -> Access {
        let clock = addr == PERIP_CLK_EN1;
        if size as usize != size_of::<u32>() || value.is_some_and(|bits| bits & !PERIP_EN1_RW != 0)
        {
            return Err(BoardFault::UnsupportedRegister {
                pc: self.pc,
                addr,
                size,
                write: value.is_some(),
                block: "поля SYSREG других блоков",
            });
        }
        let Some(bits) = value else {
            return Ok(Some(if clock { self.clk_en1 } else { self.rst_en1 }));
        };
        if clock {
            if self.clk_en1 & !bits & GDMA_GATE != 0 && chip.buses.spi_dma_active() {
                return Err(unsupported("закрытие тактов GDMA во время SPI2"));
            }
            self.clk_en1 = bits;
        } else {
            let asserted = bits & !self.rst_en1;
            if asserted & esp_sar_digital::TSENS_GATE != 0 {
                self.sar.reset_tsens();
            }
            if asserted & (SHA_GATE | SHA_DEPENDENCY_RESETS) != 0 {
                self.sha = EspSha::default();
            }
            self.rst_en1 = bits;
        }
        let rst_dma = self.rst_en1 & GDMA_GATE != 0;
        if rst_dma && chip.buses.spi_dma_active() {
            chip.buses.reset_io(false, true, now);
        }
        chip.dma
            .set_bus_control(self.clk_en1 & GDMA_GATE != 0, rst_dma);
        Ok(Some(0))
    }
}

#[derive(Clone)]
pub(super) struct Esp {
    pub pins: Pins,
    routes: [u32; 22],
    inputs: [u32; GPIO_OUT as usize],
    mux: [u32; 22],
    pin_config: [u32; 22],
    groups: [Group; 2],
    pub(super) buses: EspBuses,
    pub(super) dma: EspDma,
    channels: [LedChannel; 6],
    led_timers: [LedTimer; 4],
    led_clock: u32,
    led_source_hz: Option<(u32, u16)>,
    sdk_uart: [EspUart; 2],
    uart_time: [u64; 2],
    uart_source: [u32; 2],
    serial: [Serial; 2],
    adc_setup: u32,
    adc_irq_cleared: bool,
    adc_sampled: bool,
}

impl Default for Esp {
    fn default() -> Self {
        Self {
            pins: Pins::default(),
            routes: [GPIO_OUT; 22],         // TRM, GPIO_FUNCx_OUT_SEL reset.
            inputs: [0; GPIO_OUT as usize], // TRM, GPIO_FUNCy_IN_SEL_CFG reset.
            mux: [0; 22],
            pin_config: [0; 22],
            groups: std::array::from_fn(|_| Group::default()),
            buses: EspBuses::default(),
            dma: EspDma::default(),
            channels: std::array::from_fn(|_| LedChannel::default()),
            led_timers: std::array::from_fn(|_| LedTimer::default()),
            led_clock: 0,
            led_source_hz: None,
            sdk_uart: std::array::from_fn(|_| sdk_uart()),
            uart_time: [0; 2],
            uart_source: [3; 2], // uart_reg.h: SCLK_SEL=XTAL при сбросе.
            serial: std::array::from_fn(|_| Serial::default()),
            adc_setup: 13 << 25,
            adc_irq_cleared: false,
            adc_sampled: false,
        }
    }
}

impl Esp {
    pub(super) fn reset_boot_pads(&mut self) {
        // TRM табл. 5.12-1, MCU_SEL=0 и индивидуальные IE/WPU/DRV.
        // Уровни USB и выходы JTAG живут в отдельных блоках.
        self.mux = std::array::from_fn(|pin| {
            let drive = if matches!(pin, 18 | 19) { 3 } else { 2 };
            let active = match pin {
                2..=8 | 10 => INPUT_ENABLE,
                9 | 12..=17 | 20 => INPUT_ENABLE | PULL_UP,
                21 => PULL_UP,
                _ => 0,
            };
            (drive << 10) | active
        });
    }

    pub fn tick(
        &mut self,
        cycles: u64,
        now: u64,
        boot: Option<&BootPeripherals>,
        memory: Option<&mut EspMemory>,
    ) -> bool {
        self.buses
            .tick_with_dma(now, false, memory.map(|m| (&mut self.dma, m)));
        match boot {
            Some(_) => false, // TIMG/ADC с деревом холодной загрузки пока недоступны.
            None => {
                for port in &mut self.serial {
                    port.tick(now);
                }
                self.advance_led(u128::from(cycles), 0, (PLL80_HZ, 1), true);
                let a = self.groups[0].tick(cycles, now);
                let b = self.groups[1].tick(cycles, now);
                a || b
            }
        }
    }

    pub(super) fn advance_boot_io(
        &mut self,
        boot: &BootPeripherals,
        rates: EspClockRates,
        before: u128,
        passed: u64,
        cycle: u64,
        uart: &mut Uart,
    ) -> Result<(), BoardFault> {
        let after = boot.system.elapsed_quarter_ns();
        let (apb, den) = rates.apb_hz();
        let period = u128::from(QUARTERS_PER_SECOND / apb) * u128::from(den);
        let xtal = u128::from(QUARTERS_PER_SECOND / XTAL_HZ);
        self.advance_led(
            after / period - before / period,
            after / xtal - before / xtal,
            rates.apb_hz(),
            boot.enabled(LEDC_GATE),
        );
        // EspUart считает APB на номинальной шкале 80 МГц. Передаём ему
        // столько виртуальных периодов, сколько пришло физических фронтов APB.
        // XTAL использует ту же шкалу четвертей наносекунды без пересчёта.
        let quantum = u128::from(QUARTERS_PER_SECOND / PLL80_HZ);
        let apb_ticks = after / period - before / period;
        let first = uart.done.len();
        for (port, serial) in self.sdk_uart.iter_mut().enumerate() {
            if !boot.enabled(UART_GATE[port] | UART_MEM) {
                continue;
            }
            let local = self.uart_time[port];
            let from_apb = self.uart_source[port] == 1;
            let delta = if from_apb {
                apb_ticks * quantum
            } else {
                after - before
            };
            let end = u64::try_from(u128::from(local) + delta)
                .map_err(|_| invalid("время UART вне шкалы"))?;
            serial.tick(end)?;
            self.uart_time[port] = end;
            for (at, byte) in serial.take_tx() {
                let local_offset = u128::from(at - local);
                let absolute = if from_apb {
                    (before / period + local_offset.div_ceil(quantum)) * period
                } else {
                    before + local_offset
                };
                let offset =
                    (absolute - before).div_ceil(u128::from(rates.cpu_cycle_quarters())) as u64;
                debug_assert!(offset <= passed);
                uart.done.push(crate::OutputEvent {
                    cycle: cycle + offset,
                    output: crate::BoardOutput::Uart {
                        port: port as u32,
                        byte,
                    },
                });
            }
        }
        uart.done[first..].sort_by_key(|event| event.cycle);
        Ok(())
    }

    fn advance_led(&mut self, apb: u128, xtal: u128, apb_hz: (u32, u16), enabled: bool) {
        self.led_source_hz = enabled
            .then_some(match self.led_clock & 3 {
                1 => Some(apb_hz),
                3 => Some((XTAL_HZ, 1)),
                _ => None,
            })
            .flatten();
        if self.led_source_hz.is_some() {
            let ticks = if self.led_clock & 3 == 1 { apb } else { xtal };
            let overflows = self.led_timers.each_mut().map(|timer| timer.tick(ticks));
            for ch in &mut self.channels {
                if let Some((cfg, hpoint, duty)) = ch.pending
                    && overflows[(cfg & 3) as usize]
                {
                    ch.cfg = cfg;
                    ch.active = duty;
                    ch.active_hpoint = hpoint;
                    ch.start = true;
                    ch.pending = None;
                    ch.duty_cfg &= !LED_DUTY_START;
                }
            }
        }
    }

    fn mux_func(&self, pin: usize) -> u32 {
        self.mux[pin] >> MUX_FUNC_SHIFT & 7
    }

    fn direct_signal(&self, pin: usize) -> Option<u32> {
        // io_mux_reg.h, FUNC_U0{RX,TX}D_U0{RX,TX}D и esp_hal_gpspi/esp32c3/include/soc/spi_pins.h.
        match (self.mux_func(pin), pin) {
            (0, 20 | 21) => Some(UART_DATA[0]),
            (0, 0..=3 | 8..=11) => Some(GPIO_OUT), // TRM табл. 5.12-1.
            (2, 2) => Some(SPI_MISO),
            (2, 6) => Some(SPI_CLOCK),
            (2, 7) => Some(SPI_MOSI),
            (2, 10) => Some(SPI_CS0),
            _ => None,
        }
    }

    fn input_pad(&self, signal: u32) -> Option<usize> {
        let route = self.inputs[signal as usize];
        if route & MATRIX_IN != 0 {
            let pin = (route & INPUT_PAD) as usize;
            return self
                .mux
                .get(pin)
                .filter(|mux| **mux & INPUT_ENABLE != 0)
                .map(|_| pin);
        }
        (0..self.mux.len()).find(|&pin| {
            pin != 21
                && self.mux[pin] & INPUT_ENABLE != 0
                && self.direct_signal(pin) == Some(signal)
        })
    }

    fn output_route(&self, pin: usize) -> Option<(u32, bool)> {
        let route = *self.routes.get(pin)?;
        if self.mux_func(pin) != GPIO_FUNC {
            return match self.direct_signal(pin) {
                Some(GPIO_OUT) => (self.pins.enable & (1 << pin) != 0).then_some((GPIO_OUT, false)),
                Some(signal) if pin != 20 && signal != SPI_MISO => Some((signal, false)),
                _ => None,
            };
        }
        let signal = route & OUTPUT_SIGNAL;
        // TRM §5.5: SIG_GPIO_OUT берёт OE из GPIO_ENABLE. Для остальных
        // индексов выбирается GPIO_ENABLE или OE источника, затем OEN_INV.
        let gpio_oe = self.pins.enable & (1 << pin) != 0;
        let oe = if signal == GPIO_OUT || route & ENABLE_FROM_GPIO != 0 {
            gpio_oe
        } else {
            true
        };
        (oe ^ (route & ENABLE_INVERT != 0)).then_some((signal, route & OUTPUT_INVERT != 0))
    }

    fn output_pad(&self, signal: u32, open_drain: bool) -> bool {
        (0..self.mux.len()).any(|pin| {
            self.output_route(pin) == Some((signal, false))
                && (self.pin_config[pin] & PAD_OPEN_DRAIN != 0) == open_drain
        })
    }

    fn input_level(&self, pin: usize) -> bool {
        self.output_inner(pin, false)
            .unwrap_or_else(|| self.external_level(pin))
    }

    fn external_level(&self, pin: usize) -> bool {
        let pull = if self.mux[pin] & PULL_UP != 0 {
            1 << pin
        } else {
            0
        };
        self.pins.input_with_pull(pull) & (1 << pin) != 0
    }

    fn pwm_params(&self, pin: usize) -> Option<(&LedTimer, u32, u32, u32, bool)> {
        let (signal, invert) = self.output_route(pin)?;
        if self.pin_config[pin] & PAD_OPEN_DRAIN != 0 {
            return None;
        }
        let ch = self
            .channels
            .get(signal.checked_sub(LEDC_FIRST)? as usize)?;
        let timer = &self.led_timers[(ch.cfg & 3) as usize];
        let (period, _) = timer.params()?;
        if !ch.start || ch.cfg & LED_OUTPUT_ENABLE == 0 || ch.active_hpoint >= period {
            return None;
        }
        let high = ch.active >> 4;
        (high <= period).then_some((timer, period, ch.active_hpoint, high, invert))
    }

    pub fn pwm(&self, pin: u32) -> Option<PwmOutput> {
        let (hz, den) = self.led_source_hz?;
        let (timer, period, _, high, invert) = self.pwm_params(pin as usize)?;
        if timer.cfg & (LED_TIMER_RESET | LED_TIMER_PAUSE) != 0 {
            return None;
        }
        let (_, divider) = timer.params()?;
        let seconds = f64::from(period) * divider as f64 * f64::from(den) / (256.0 * f64::from(hz));
        let high = if invert { period - high } else { high };
        Some(PwmOutput::new(f64::from(high) / f64::from(period), seconds))
    }

    pub fn uart_receive(
        &mut self,
        port: u32,
        now: u64,
        bytes: &[u8],
        boot: Option<&BootPeripherals>,
    ) -> Result<InputOutcome, BoardFault> {
        let port = port as usize;
        if port >= UART_DATA.len() {
            return Err(unsupported("номер UART ESP32-C3"));
        }
        let Some(boot) = boot else {
            return self.serial[port].receive(now, bytes);
        };
        let signal = UART_DATA[port];
        if self.input_pad(signal).is_none()
            || self.inputs[signal as usize] & (MATRIX_IN | INPUT_INVERT) == MATRIX_IN | INPUT_INVERT
        {
            return Err(invalid("UART: RX требует активный маршрут без инверсии"));
        }
        if !boot.enabled(UART_GATE[port] | UART_MEM) {
            return Err(invalid("UART: нет тактов SYSREG или активен сброс"));
        }
        self.sdk_uart[port].receive(self.uart_time[port], bytes)
    }

    pub fn output(&self, pin: u32) -> Option<bool> {
        self.output_inner(pin as usize, true)
    }

    fn output_inner(&self, pin: usize, loopback: bool) -> Option<bool> {
        let (signal, invert) = self.output_route(pin)?;
        let level = if signal == GPIO_OUT {
            self.pins.out & (1 << pin) != 0
        } else if let Some((timer, period, start, high, _)) = self.pwm_params(pin) {
            (timer.ctr + period - start) % period < high
        } else if let Some(ch) = signal
            .checked_sub(LEDC_FIRST)
            .and_then(|i| self.channels.get(i as usize))
        {
            if ch.start && ch.cfg & LED_OUTPUT_ENABLE != 0 {
                return None; // Недопустимые HPOINT или DUTY.
            }
            ch.cfg & LED_IDLE != 0
        } else if loopback && LOOPBACK.contains(&signal) {
            let route = self.inputs[signal as usize];
            if route & MATRIX_IN == 0 {
                return None;
            }
            let value = match route & INPUT_PAD {
                INPUT_ONE => true,
                INPUT_ZERO => false,
                _ => self.input_level(self.input_pad(signal)?),
            };
            value ^ (route & INPUT_INVERT != 0)
        } else {
            return None; // У кадровых шин нет модели мгновенного уровня на выводе.
        } ^ invert;
        (self.pin_config[pin] & PAD_OPEN_DRAIN == 0 || !level).then_some(level)
    }

    pub(super) fn pad_is_flash(&self, pin: u32) -> bool {
        let pin = pin as usize;
        FLASH_PADS.contains(&pin) && self.mux_func(pin) == 0
    }

    pub(super) fn passive_pin(&self, pin: u32) -> Option<crate::GpioMode> {
        use crate::GpioMode::*;
        let pin = pin as usize;
        let mux = *self.mux.get(pin)?;
        if self.output_route(pin).is_some_and(|(s, _)| {
            UART_DATA.contains(&s)
                || I2C_LINES.contains(&s)
                || (SPI_CLOCK..SPI_CS0 + 6).contains(&s)
        }) {
            return None;
        }
        match mux & (PULL_UP | PULL_DOWN) {
            PULL_UP => Some(PullUp),
            PULL_DOWN => Some(PullDown),
            0 if mux & INPUT_ENABLE != 0 => Some(Input),
            0 => Some(HighImpedance),
            _ => None,
        }
    }

    pub(super) fn digital_wakeup_level(&self, pin: usize) -> Option<bool> {
        let config = self.pin_config[pin];
        (config & PIN_WAKE_ENABLE != 0).then(|| config & PIN_INT_TYPE == GPIO_HIGH << PIN_INT_SHIFT)
    }

    pub(super) fn sleep_pad_mode(&self, pin: u32, mode: EspSleepMode) -> Option<GpioMode> {
        let mux = *self.mux.get(pin as usize)?;
        // TRM §§5.11, 9.4. В Light-sleep IO MUX может выбрать настройки сна.
        // В Deep-sleep цифровые маршруты и удержание выходов пока не наблюдаемы.
        let selected = mux & SLEEP_SELECT != 0;
        if mode == EspSleepMode::Light {
            if self.output_route(pin as usize).is_some()
                && (!selected || mux & SLEEP_OUTPUT_ENABLE != 0)
            {
                // У спящего LEDC или пакетной линии нет доказанного уровня.
                if self.output_route(pin as usize)?.0 != GPIO_OUT {
                    return None;
                }
                if let Some(level) = self.output(pin) {
                    return Some(GpioMode::Output(level));
                }
            } else if selected && mux & SLEEP_OUTPUT_ENABLE != 0 {
                return None;
            }
        } else if pin as usize >= RTC_GPIO_COUNT {
            return None;
        }
        let (up, down, input) = if selected {
            (SLEEP_PULL_UP, SLEEP_PULL_DOWN, SLEEP_INPUT_ENABLE)
        } else {
            (PULL_UP, PULL_DOWN, INPUT_ENABLE)
        };
        match mux & (up | down) {
            bits if bits == up => Some(GpioMode::PullUp),
            bits if bits == down => Some(GpioMode::PullDown),
            0 if mux & input != 0 => Some(GpioMode::Input),
            0 => Some(GpioMode::HighImpedance),
            _ => None,
        }
    }

    pub(super) fn active_wake_pins(
        &self,
        low: u32,
        high: u32,
        mode: EspSleepMode,
    ) -> Result<u32, BoardFault> {
        let mut candidate = low | high;
        let mut active = 0;
        while candidate != 0 {
            let pin = candidate.trailing_zeros();
            let bit = 1 << pin;
            candidate &= !bit;
            let mux = self.mux[pin as usize];
            let input_enabled = if mux & SLEEP_SELECT != 0 {
                mux & SLEEP_INPUT_ENABLE != 0
            } else {
                mux & INPUT_ENABLE != 0
            };
            if !input_enabled {
                return Err(unsupported(&format!(
                    "GPIO{pin}: вход пробуждения выключен"
                )));
            }
            let level = match self.sleep_pad_mode(pin, mode) {
                Some(GpioMode::Input | GpioMode::PullUp | GpioMode::PullDown)
                    if self.pins.driven & bit != 0 =>
                {
                    self.pins.input & bit != 0
                }
                Some(GpioMode::PullUp) => true,
                Some(GpioMode::PullDown) => false,
                _ => {
                    return Err(unsupported(&format!(
                        "GPIO{pin}: уровень пробуждения не определён"
                    )));
                }
            };
            if level == (high & bit != 0) {
                active |= bit;
            }
        }
        Ok(active)
    }

    fn check_i2c_routes(&self) -> Result<(), BoardFault> {
        for signal in I2C_LINES {
            let Some(pin) = self.input_pad(signal) else {
                return Err(invalid("I²C: SDA/SCL без входного маршрута"));
            };
            if self.output_route(pin) != Some((signal, false))
                || self.inputs[signal as usize] & INPUT_INVERT != 0
                || self.pin_config[pin] & PAD_OPEN_DRAIN == 0
            {
                return Err(invalid("I²C: SDA/SCL требуют двунаправленный open drain"));
            }
        }
        Ok(())
    }

    fn check_spi_routes(&self) -> Result<(), BoardFault> {
        let cs = self.buses.spi_chip_select()?;
        if ![SPI_CLOCK, SPI_MOSI, SPI_CS0 + u32::from(cs)]
            .into_iter()
            .all(|s| self.output_pad(s, false))
            || self.input_pad(SPI_MISO).is_none()
            || self.inputs[SPI_MISO as usize] & (MATRIX_IN | INPUT_INVERT)
                == MATRIX_IN | INPUT_INVERT
        {
            return Err(invalid(
                "SPI2: нужны маршруты SCLK, MOSI, MISO и выбранного CS",
            ));
        }
        Ok(())
    }

    pub fn access(
        &mut self,
        addr: u32,
        v: Option<u32>,
        now: u64,
        uart: &mut Uart,
        adc: &mut Adc,
        boot: Option<&BootPeripherals>,
        memory: Option<&mut EspMemory>,
    ) -> Access {
        let o = addr & 0xfff;
        let block = addr & !0xfff;
        if boot.is_some()
            && v.is_some()
            && self.buses.active()
            && (block == 0x6000_9000
                || block == 0x6000_4000
                    && matches!(o, 0x20 | 0x24 | 0x28 | 0x74..=0xc8 | 0x154..=0x350 | 0x554..=0x5a8))
        {
            return Err(unsupported(
                "изменение маршрутов GPIO во время пакетного обмена",
            ));
        }
        match block {
            0x6000_4000 => match o {
                4 => rw(v, &mut self.pins.out, GPIO_MASK),
                8 => command(v, |v| self.pins.out |= v & GPIO_MASK),
                0xc => command(v, |v| self.pins.out &= !v),
                0x20 => rw(v, &mut self.pins.enable, GPIO_MASK),
                0x24 => command(v, |v| self.pins.enable |= v & GPIO_MASK),
                0x28 => command(v, |v| self.pins.enable &= !v),
                // TRM рег. 5.12: очистка защёлок GPIO_STATUS. Цифровые
                // прерывания GPIO_PINn.INT_ENA здесь выключены: запись в
                // INT_ENA пока отклоняется, поэтому очищать нечего.
                0x4c if boot.is_some() => command(v, |_| {}),
                0x3c => ro(
                    v,
                    (0..22).fold(0, |all, i| {
                        let high = self.mux[i] & INPUT_ENABLE != 0
                            && self
                                .output(i as u32)
                                .unwrap_or_else(|| self.external_level(i));
                        all | (u32::from(high) << i)
                    }),
                ),
                0x74..=0xc8 => {
                    if v.is_some_and(|bits| {
                        let kind = (bits & PIN_INT_TYPE) >> PIN_INT_SHIFT;
                        let wake = bits & PIN_WAKE_ENABLE != 0;
                        if wake {
                            !matches!(kind, GPIO_LOW | GPIO_HIGH)
                        } else {
                            !matches!(kind, 0 | GPIO_LOW | GPIO_HIGH)
                        }
                    }) {
                        return Err(unsupported(
                            "GPIO_PIN: поддерживается пробуждение по уровню",
                        ));
                    }
                    checked_rw(
                        v,
                        &mut self.pin_config[((o - 0x74) / 4) as usize],
                        PAD_OPEN_DRAIN | PIN_WAKE_ENABLE | PIN_INT_TYPE,
                        "GPIO_PIN: синхронизация и прерывания",
                    )
                }
                0x154..=0x350 => {
                    if v.is_some_and(|value| {
                        value & MATRIX_IN != 0
                            && !matches!(value & INPUT_PAD, 0..=21 | INPUT_ONE | INPUT_ZERO)
                    }) {
                        return Err(invalid("GPIO Matrix: недоступный источник входа"));
                    }
                    checked_rw(
                        v,
                        &mut self.inputs[((o - 0x154) / 4) as usize],
                        MATRIX_IN | INPUT_INVERT | INPUT_PAD,
                        "GPIO_FUNCy_IN_SEL_CFG",
                    )
                }
                0x554..=0x5a8 => {
                    if v.is_some_and(|v| !matches!(v & OUTPUT_SIGNAL, 6 | 9 | 45..=50 | 53..=54 | 63..=73 | 97..=100 | GPIO_OUT)) {
                        return Err(unsupported("выбранный выход GPIO Matrix"));
                    }
                    checked_rw(
                        v,
                        &mut self.routes[((o - 0x554) / 4) as usize],
                        ENABLE_INVERT | ENABLE_FROM_GPIO | OUTPUT_INVERT | OUTPUT_SIGNAL,
                        "GPIO_FUNCx_OUT_SEL_CFG",
                    )
                }
                _ => Ok(None),
            },
            0x6000_9000 if (4..=88).contains(&o) => {
                let pin = (o / 4 - 1) as usize;
                // Функции платы: TRM §5.12, io_mux_reg.h. Нижние биты хранят
                // конфигурацию сна, которую активирует RTC_CNTL.
                if let Some(value) = v {
                    let function = value >> MUX_FUNC_SHIFT & 7;
                    let supported = function == GPIO_FUNC
                        || matches!((function, pin), (0, 0..=3 | 8..=11 | 20 | 21) | (2, 2 | 6 | 7 | 10))
                        || (boot.is_some() && function == 0 && FLASH_PADS.contains(&pin));
                    // gpio_sleep_{input,output}_disable изменяют конфигурацию
                    // сна и сохраняют выбранную при сбросе прямую функцию.
                    let sleep_only =
                        boot.is_some() && (value ^ self.mux[pin]) & !PAD_SLEEP_FIELDS == 0;
                    if !supported && !sleep_only {
                        return Err(unsupported("IO MUX: выбранная прямая функция"));
                    }
                    if value & (PULL_UP | PULL_DOWN) == (PULL_UP | PULL_DOWN) {
                        return Err(unsupported("IO MUX: одновременно включены обе подтяжки"));
                    }
                }
                checked_rw(v, &mut self.mux[pin], PAD_ACTIVE_MASK, "IO MUX")
            }
            0x6001_f000 => self.groups[0].access(o, v, now),
            0x6002_0000 => self.groups[1].access(o, v, now),
            0x6001_3000 | 0x6002_4000 => {
                let i2c = block == 0x6001_3000;
                if let Some(boot) = boot {
                    let gate = if i2c { I2C_GATE } else { SPI_GATE };
                    // INT_ST читается матрицей IRQ даже при остановленном блоке.
                    let irq = if i2c { 0x2c } else { 0x40 };
                    if !boot.enabled(gate) {
                        return if o == irq && v.is_none() {
                            ro(v, 0)
                        } else {
                            Err(invalid("I²C/SPI: нет тактов SYSREG или активен сброс"))
                        };
                    }
                    // i2c_reg.h I2C_TRANS_START, spi_reg.h SPI_USR.
                    if let Some(bits) = v {
                        if i2c && o == 4 && bits & (1 << 5) != 0 {
                            self.check_i2c_routes()?;
                        } else if !i2c && o == 0 && bits & (1 << 24) != 0 {
                            self.check_spi_routes()?;
                        }
                    }
                }
                let cpu = boot.map_or((PLL80_HZ, 1), |b| b.system.clocks().cpu_hz());
                if !i2c
                    && o == 0
                    && v.is_some_and(|bits| bits & (1 << 24) != 0)
                    && self.buses.spi_uses_dma()
                    && boot.is_some_and(|b| !b.enabled(SPI_DMA_GATE))
                {
                    return Err(invalid("SPI2: выключен затвор DMA в SYSREG"));
                }
                self.buses.access(
                    if i2c { 0 } else { 2 },
                    o,
                    v,
                    now,
                    cpu,
                    boot.is_none(),
                    memory.map(|m| (&mut self.dma, m)),
                )
            }
            0x6001_9000 => {
                if boot.is_some_and(|b| !b.enabled(LEDC_GATE)) {
                    return Err(invalid("LEDC: нет тактов SYSREG или активен сброс"));
                }
                if o < 0x78 {
                    let chan = &mut self.channels[(o / 0x14) as usize];
                    return match o % 0x14 {
                        0 => {
                            if let Some(v) = v {
                                if v & !31 != 0 {
                                    return Err(unsupported(
                                        "LEDC CONF0: только постоянная скважность",
                                    ));
                                }
                                chan.shadow = v & 15;
                                if v & LED_CHANNEL_UPDATE != 0 {
                                    if boot.is_some() {
                                        // TRM §32.3.3: сигнал и HPOINT/DUTY обновляются
                                        // при переполнении выбранного таймера.
                                        chan.pending = Some((chan.shadow, chan.hpoint, chan.duty));
                                    } else {
                                        chan.cfg = chan.shadow;
                                        chan.active_hpoint = chan.hpoint;
                                    }
                                }
                                Ok(Some(0))
                            } else {
                                ro(v, chan.shadow)
                            }
                        }
                        4 => rw(v, &mut chan.hpoint, 0x3fff),
                        8 => {
                            if v.is_some_and(|v| v & 15 != 0) {
                                return Err(unsupported("дробная скважность LEDC"));
                            }
                            rw(v, &mut chan.duty, 0x7ffff)
                        }
                        12 => {
                            if let Some(v) = v {
                                // ledc_ll_set_duty_*: один цикл с DUTY_SCALE=0
                                // штатно задаёт постоянную скважность.
                                if v & 0x3ff != 0 || (v >> 20 & 0x3ff) > 1 {
                                    return Err(unsupported("плавное изменение скважности LEDC"));
                                }
                                chan.duty_cfg = v;
                                if boot.is_none() && v & LED_DUTY_START != 0 {
                                    chan.start = true;
                                    chan.active = chan.duty;
                                    chan.active_hpoint = chan.hpoint;
                                }
                                Ok(Some(0))
                            } else {
                                ro(v, chan.duty_cfg)
                            }
                        }
                        _ => ro(v, chan.active),
                    };
                }
                match o {
                    0xa0..=0xbc => {
                        let timer = &mut self.led_timers[((o - 0xa0) / 8) as usize];
                        if o & 4 != 0 {
                            return ro(v, timer.ctr);
                        }
                        if let Some(v) = v {
                            if v & !(LED_TIMER_UPDATE | 0xff_ffff) != 0
                                || v & 15 > 14
                                || v & LED_TIMER_TICK_SEL != 0
                            {
                                return Err(unsupported("LEDC TIMER CONF"));
                            }
                            let configured = v & 0x00ff_ffff;
                            let mode = LED_TIMER_RESET | LED_TIMER_PAUSE;
                            if self.led_clock & 3 == 2 && configured & 15 != 0 {
                                return Err(unsupported("LEDC: частота RC_FAST не задана"));
                            }
                            timer.shadow = configured;
                            // RESET/PAUSE управляют счётчиком сразу. Новый делитель
                            // работающего таймера защёлкивается на переполнении.
                            let stopped =
                                timer.cfg & LED_TIMER_RESET != 0 || timer.params().is_none();
                            timer.cfg = (timer.cfg & !mode) | (configured & mode);
                            if let Some(pending) = &mut timer.pending {
                                *pending = (*pending & !mode) | (configured & mode);
                            }
                            if v & LED_TIMER_UPDATE != 0 {
                                if boot.is_none() {
                                    timer.cfg = configured;
                                    if let Some((period, div)) = timer.params() {
                                        timer.ctr %= period;
                                        timer.phase %= div as u64;
                                    }
                                } else if stopped || configured & LED_TIMER_RESET != 0 {
                                    timer.cfg = configured;
                                    timer.pending = None;
                                    timer.ctr = 0;
                                    timer.phase = 0;
                                } else {
                                    timer.pending = Some(configured);
                                }
                            }
                            if timer.cfg & LED_TIMER_RESET != 0 {
                                timer.ctr = 0;
                                timer.phase = 0;
                            }
                            Ok(Some(0))
                        } else {
                            ro(v, timer.shadow)
                        }
                    }
                    0xd0 => {
                        if v.is_some_and(|v| {
                            let source = v & 3;
                            boot.is_none() && source > 1
                                || source == 2
                                    && self.led_timers.iter().any(|timer| timer.params().is_some())
                        }) {
                            return Err(unsupported("источник тактов LEDC"));
                        }
                        checked_rw(v, &mut self.led_clock, 0x8000_0003, "LEDC CONF")
                    }
                    _ => Ok(None),
                }
            }
            0x6004_0000 => match o {
                0x20 => {
                    if let Some(v) = v {
                        if v & !0xbf80_0000 != 0 {
                            return Err(unsupported(
                                "ESP32-C3 ADC2 и режимы кроме программного одиночного",
                            ));
                        }
                        if v & 1 << 29 != 0 && self.adc_setup & 1 << 29 == 0 {
                            let channel = (v >> 25 & 15) as usize;
                            if v & 1 << 31 == 0 || channel >= 5 || adc.deadline.is_some() {
                                return Err(invalid(
                                    "ADC1 выключен, занят или выбран недопустимый канал",
                                ));
                            }
                            let attenuation = (v >> 23 & 3) as usize;
                            let volts = adc.volts[channel]
                                .ok_or_else(|| invalid("не задан вход ADC1 ESP32-C3"))?;
                            // Datasheet таблица 5-6: границы откалиброванного рабочего диапазона.
                            if !(0.0..=[0.75, 1.05, 1.3, 2.5][attenuation]).contains(&volts) {
                                return Err(invalid(
                                    "ADC1 ESP32-C3 вне диапазона выбранного ослабления",
                                ));
                            }
                            // TRM §34.2.4: номинальные Vref и коэффициенты ослабления.
                            // Оценка raw-кода, калибровка eFuse и нелинейность отсутствуют.
                            let gain = [1.0, 0.75, 0.5, 0.25][attenuation];
                            adc.result = (volts * gain / 1.1 * 4095.0).floor() as u32;
                            // Таблица 5-5 даташита приводит до 100 ksps для DIG ADC.
                            // Оценка 10 мкс на отсчёт, не гарантия латентности one-shot.
                            adc.ready = false;
                            adc.deadline = Some(now + 800);
                            self.adc_irq_cleared = false;
                            self.adc_sampled = true;
                        }
                        self.adc_setup = v;
                        Ok(Some(0))
                    } else {
                        ro(v, self.adc_setup)
                    }
                }
                0x2c if self.adc_sampled && adc.ready => ro(v, adc.result),
                0x2c => Err(invalid("результат SAR ADC1 ещё не получен")),
                0x44 => ro(
                    v,
                    u32::from(self.adc_sampled && adc.ready && !self.adc_irq_cleared) << 31,
                ),
                0x4c => command(v, |v| {
                    if v >> 31 != 0 && adc.ready {
                        self.adc_irq_cleared = true;
                    }
                }),
                _ => Ok(None),
            },
            0x6000_0000 | 0x6001_0000 => {
                let i = usize::from(addr >= 0x6001_0000);
                if let Some(boot) = boot {
                    if o == 8
                        && v.is_none()
                        && (!boot.enabled(UART_GATE[i] | UART_MEM) || !self.sdk_uart[i].irq())
                    {
                        return ro(v, 0);
                    }
                    if !boot.enabled(UART_GATE[i] | UART_MEM) {
                        // ESP-IDF esp_driver_uart/src/uart.c, uart_module_enable:
                        // сначала открывает APB, читает исходный CLK_CONF и
                        // снимает override питания в MEM_CONF, потом снимает
                        // сброс ядра. FIFO и состояние IRQ требуют рабочего UART.
                        // Смещения и значения сброса — uart_reg.h, гл. 26 TRM.
                        if boot.clk_io & (UART_GATE[i] | UART_MEM) == (UART_GATE[i] | UART_MEM)
                            && boot.rst_io & UART_GATE[i] != 0
                        {
                            let at = self.uart_time[i];
                            let serial = &mut self.sdk_uart[i];
                            if matches!(o, 0x60 | 0x78) && v.is_none() {
                                return serial.access(o, None, at);
                            }
                            if o == 0x60 && serial.access(o, None, at)? == v {
                                return Ok(Some(0));
                            }
                        }
                        return Err(invalid("UART: нет тактов SYSREG или активен сброс"));
                    }
                    let at = self.uart_time[i];
                    if o == 0 && v.is_some() {
                        if !self.output_pad(UART_DATA[i], false) {
                            return Err(invalid("UART: TX требует активный маршрут без инверсии"));
                        }
                    }
                    let serial = &mut self.sdk_uart[i];
                    // uart_reg.h: UART_ID.HIGH_SPEED и UART_UPDATE разрешают
                    // перенос полей CLK_CONF в рабочий домен.
                    let source = match (o, v) {
                        (0x78, Some(bits))
                            if serial.access(0x80, None, at)?.unwrap() & (1 << 30) != 0 =>
                        {
                            Some(bits >> 20 & 3)
                        }
                        (0x80, Some(bits)) if bits & (3 << 30) != 0 => {
                            Some(serial.access(0x78, None, at)?.unwrap() >> 20 & 3)
                        }
                        _ => None,
                    };
                    if source.is_some_and(|next| next != self.uart_source[i])
                        && serial.access(0x1c, None, at)?.unwrap() & 0x3ff != 0
                    {
                        return Err(unsupported("смена источника UART при непустом RX FIFO"));
                    }
                    let result = serial.access(o, v, at).map_err(|fault| {
                        let operation = if v.is_some() {
                            "запись"
                        } else {
                            "чтение"
                        };
                        match fault {
                            BoardFault::Misconfigured(reason) => BoardFault::Misconfigured(
                                format!("{operation} UART MMIO {addr:#010x}: {reason}"),
                            ),
                            BoardFault::Unsupported(reason) => BoardFault::Unsupported(format!(
                                "{operation} UART MMIO {addr:#010x}: {reason}"
                            )),
                            fault => fault,
                        }
                    })?;
                    if let Some(source) = source {
                        self.uart_source[i] = source;
                    }
                    return Ok(result);
                }
                let serial = &mut self.serial[i];
                let tx = &mut uart.ports[i];
                if v.is_some() && !serial.wire.is_empty() && matches!(o, 0x14 | 0x20 | 0x78) {
                    return Err(unsupported("изменение UART во время приёма кадра"));
                }
                match o {
                    0 if v.is_some() => {
                        let signal = if i == 0 { 6 } else { 9 };
                        if serial.config & 0x1000_003f != 0x1000_001c
                            || !self.routes.iter().enumerate().any(|(pin, r)| {
                                r & 255 == signal
                                    && self.mux[pin] >> 12 & 7 == 1
                                    && (r & 0x200 == 0 || self.pins.enable & (1 << pin) != 0)
                            })
                        {
                            return Err(invalid("UART требует 8N1 и маршрут TX через GPIO Matrix"));
                        }
                        tx.start(
                            now,
                            serial.frame_cycles_x16(true)?.div_ceil(16),
                            v.unwrap(),
                            UART_FIFO,
                        )
                    }
                    0 => serial
                        .rx
                        .pop_front()
                        .map(|b| Some(u32::from(b)))
                        .ok_or_else(|| invalid("UART: RX FIFO пуст")),
                    4 => ro(v, serial.raw()),
                    8 => ro(v, serial.raw() & serial.enable),
                    0xc => checked_rw(v, &mut serial.enable, RX_FULL | RX_OVERFLOW, "UART INT_ENA"),
                    0x10 => command(v, |bits| {
                        if bits & RX_OVERFLOW != 0 {
                            serial.overflow = false;
                        }
                    }),
                    0x14 => rw(v, &mut serial.div, 0x00f0_0fff),
                    0x1c => ro(v, ((tx.queued() as u32) << 16) | serial.rx.len() as u32),
                    0x20 => {
                        let result = checked_rw(
                            v,
                            &mut serial.config,
                            0x1202_003f,
                            "UART CONF0: только 8N1",
                        )?;
                        if v.is_some_and(|value| value & (1 << 17) != 0) {
                            serial.rx.clear();
                            serial.config &= !(1 << 17);
                        }
                        Ok(result)
                    }
                    0x24 => {
                        if v.is_some_and(|n| n >= UART_FIFO as u32) {
                            return Err(unsupported("UART CONF1: только порог RX FIFO"));
                        }
                        checked_rw(v, &mut serial.threshold, 0x1ff, "UART CONF1")
                    }
                    0x6c => ro(v, u32::from(tx.busy()) << 4),
                    0x78 => checked_rw(v, &mut serial.clock, 0x037f_f000, "UART CLK_CONF"),
                    _ => Ok(None),
                }
            }
            _ => Ok(None),
        }
    }
}

impl BootPeripherals {
    pub(super) fn advance_timg(
        &mut self,
        elapsed: u128,
        rates: EspClockRates,
    ) -> Option<(usize, MwdtReset)> {
        for (group, gate) in esp_timg::GATES.into_iter().enumerate() {
            if self.enabled(gate)
                && let Some(action) = self.timg[group].advance(elapsed, rates)
            {
                return Some((group, action));
            }
        }
        None
    }

    fn enabled(&self, mask: u32) -> bool {
        self.clk_io & mask == mask && self.rst_io & mask == 0
    }
}

fn sdk_uart() -> EspUart {
    EspUart::new(QUARTERS_PER_SECOND, PLL80_HZ, XTAL_HZ, None)
        .expect("номиналы TRM §6.2 и целочисленная шкала часов")
}
