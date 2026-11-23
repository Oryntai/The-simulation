//! Прошивка контроллера локтевого экзоскелета для VMCU-C3.
//!
//! Контракт с хостом (физикой приложения) — `crates/sim-mcu/MEMORY_MAP.md`,
//! раздел «exo-ctrl», и `sim_mcu::exo` на стороне хоста.
//!
//! Структура: прерывание таймера 10 кГц — токовый контур и защиты, каждый 10-й
//! тик — внешний контур 1 кГц ([`control::Ctl`]); внешнее прерывание радио —
//! пакеты в очередь; основной цикл — команды, телеметрия каждые 20 мс,
//! индикатор, сторож, `wfi`.
#![no_std]
#![no_main]

mod config;
mod control;

use control::{Ctl, PIN_DIR, PIN_EN, PIN_LED, PWM_MOTOR, TICK_HZ};
use vmcu_hal::interrupt::{self, Global};
use vmcu_hal::{gpio, intc, pwm, radio, rt, sys, timer, wdt};

vmcu_hal::entry!(main);

static CTL: Global<Ctl> = Global::new(Ctl::new());

/// Очередь принятых пакетов (заполняет прерывание радио, разбирает основной цикл).
struct RxQueue {
    pkt: [[u8; 16]; 4],
    len: [u8; 4],
    head: usize,
    count: usize,
}

impl RxQueue {
    fn push(&mut self, data: &[u8], len: usize) {
        if self.count == self.pkt.len() {
            return; // переполнение: пакет теряется
        }
        let slot = (self.head + self.count) % self.pkt.len();
        let n = len.min(16);
        self.pkt[slot][..n].copy_from_slice(&data[..n]);
        self.len[slot] = len.min(255) as u8;
        self.count += 1;
    }

    fn pop(&mut self) -> Option<([u8; 16], usize)> {
        if self.count == 0 {
            return None;
        }
        let r = (self.pkt[self.head], self.len[self.head] as usize);
        self.head = (self.head + 1) % self.pkt.len();
        self.count -= 1;
        Some(r)
    }
}

static RXQ: Global<RxQueue> = Global::new(RxQueue {
    pkt: [[0; 16]; 4],
    len: [0; 4],
    head: 0,
    count: 0,
});

/// Период телеметрии: 20 мс = 200 тиков.
const TLM_TICKS: u32 = TICK_HZ / 50;
/// Команда телефона: 0xC1, режим, поддержка %, цель×10 (u16 LE), груз×10 (u16 LE), seq.
const CMD_TAG: u8 = 0xc1;
const CMD_LEN: usize = 8;

fn timer_isr() {
    CTL.with(|c| c.fast_tick());
}

fn radio_isr() {
    let mut buf = [0u8; 16];
    while let Some((len, _rssi)) = radio::recv(&mut buf) {
        RXQ.with(|q| q.push(&buf, len));
    }
}

fn handle_packet(p: &[u8; 16], len: usize) {
    if len != CMD_LEN || p[0] != CMD_TAG {
        return;
    }
    let target10 = u16::from_le_bytes([p[3], p[4]]);
    let payload10 = u16::from_le_bytes([p[5], p[6]]);
    CTL.with(|c| c.apply_command(p[1], p[2], target10, payload10, p[7]));
}

/// Имитация зависшего основного цикла (тест сторожа): бесконечный цикл без «кормления».
fn hang() -> ! {
    #[allow(clippy::empty_loop)]
    loop {}
}

fn main() -> ! {
    let (cfg, from_block) = config::load();
    let after_watchdog = sys::reset_reason() == sys::RESET_WATCHDOG;

    // Выходы: драйвер выключен до первой команды.
    gpio::set_low(PIN_EN);
    gpio::set_low(PIN_DIR);
    gpio::set_low(PIN_LED);
    gpio::set_output(PIN_EN);
    gpio::set_output(PIN_DIR);
    gpio::set_output(PIN_LED);
    pwm::set_duty(PWM_MOTOR, 0);
    pwm::configure(PWM_MOTOR, 20_000);

    wdt::start(cfg.wdt_timeout_ms.saturating_mul(1000));
    rt::set_timer_handler(timer_isr);
    rt::set_external_handler(radio_isr);
    // Оценка обмотки переживает сброс сторожем/программный (ОЗУ не обнулялось),
    // но не включение питания.
    let retained = if sys::reset_reason() != sys::RESET_POWER_ON {
        control::keep_load()
    } else {
        None
    };
    CTL.with(|c| c.init(cfg, from_block, after_watchdog, retained));
    timer::enable_irq();
    intc::enable(intc::RADIO_RX);
    interrupt::enable();

    let mut tlm_tick = 0u32;
    let mut seq = 0u8;
    let mut led = false;
    loop {
        wdt::kick();
        while let Some((p, len)) = RXQ.with(|q| q.pop()) {
            handle_packet(&p, len);
        }
        let tick = CTL.with(|c| c.tick);
        if tick.wrapping_sub(tlm_tick) >= TLM_TICKS {
            tlm_tick = tlm_tick.wrapping_add(TLM_TICKS);
            let pkt = CTL.with(|c| c.telemetry(seq));
            radio::send(&pkt);
            seq = seq.wrapping_add(1);
        }
        // Индикатор: горит — всё в порядке; мигает 5 Гц (100 мс / 100 мс) — неисправность.
        let fault = CTL.with(|c| c.any_fault());
        let want = !fault || (tick / (TICK_HZ / 10)).is_multiple_of(2);
        if want != led {
            gpio::write(PIN_LED, want);
            led = want;
        }
        if cfg.test_hang_ms != 0 && !after_watchdog && tick / 10 >= cfg.test_hang_ms {
            hang();
        }
        interrupt::wait();
    }
}
