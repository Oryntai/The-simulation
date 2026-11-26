//! Проверки автомата сна через инструкции CPU, RTC_CNTL и входы платы.

use sim_boards::{
    BoardFault, BoardId, BoardInput, BoardOutput, BoardSim, EspBootOptions, EspBootPosition,
    EspResetProfile, EspResetValueKind, EspSleepMode, EspWake, InputEvent, ResetCause, esp_image,
    new_board,
};

const FLASH: &[u8] = include_bytes!("data/esp_peripherals/flash.bin");
const QEMU_EFUSE: &[u8] = include_bytes!("data/esp_peripherals/qemu-efuse.bin");
const CRC_PROFILE: &str =
    include_str!("../firmware-tests/esp_sdk/reference/esp32c3-qemu-rtc-crc.toml");
const QEMU_TRACE: &str = include_str!("data/esp_peripherals/qemu.trace");

// TRM v1.4, §3.3.2, §9.7 и ESP-IDF v6.1.1, rtc_cntl_reg.h.
const ROM: u32 = 0x4000_0000;
const ROM_SIZE: usize = 384 * 1024;
const SRAM: u32 = 0x3fc8_0000;
const RTC_RAM: u32 = 0x5000_0000;
const RTC: u32 = 0x6000_8000;
const ALARM: u32 = RTC + 4;
const ALARM_HIGH: u32 = RTC + 8;
const TIME_UPDATE: u32 = RTC + 0x0c;
const TIME_LOW: u32 = RTC + 0x10;
const STATE: u32 = RTC + 0x18;
const TIMER5: u32 = RTC + 0x2c; // TRM рег. 9.10, RTC_CNTL_MIN_SLP_VAL.
const RESET_STATE: u32 = RTC + 0x38;
const SOURCES: u32 = RTC + 0x3c;
const INT_ENABLE: u32 = RTC + 0x40;
const INT_RAW: u32 = RTC + 0x44;
const INT_ST: u32 = RTC + 0x48;
const INT_CLR: u32 = RTC + 0x4c;
const RTC_STORE0: u32 = RTC + 0x50;
const REJECT_CONF: u32 = RTC + 0x68;
const EXT_WAKEUP: u32 = RTC + 0x64;
const POWER: u32 = RTC + 0x88;
const WDT_CONFIG0: u32 = RTC + 0x90;
const WDT_STAGE0: u32 = RTC + 0x94;
const WDT_WPROTECT: u32 = RTC + 0xa8;
const REJECT_CAUSE: u32 = RTC + 0xf0;
const WAKE_CAUSE: u32 = RTC + 0xf8;
const RTC_GPIO_WAKE: u32 = RTC + 0x110;
const CRC_CONFIG: u32 = 0x600c_0048; // system_reg.h, TRM рег. 16.11.
const CRC_RESULT: u32 = 0x600c_004c; // SYSTEM_RTC_FASTMEM_CRC_REG, TRM рег. 16.12.
const CRC_RESET: u32 = 0x7ff << 20; // RTC_MEM_CRC_LEN при сбросе.
const CRC_START: u32 = 1 << 8;
const CRC_FINISH: u32 = 1 << 31;
const DIGITAL_GPIO: u32 = 0x6000_4074; // GPIO_PIN0_REG, gpio_reg.h.
const MUX_GPIO: u32 = 0x6000_9004; // IO_MUX_GPIO0_REG, io_mux_reg.h.
const MUX_GPIO_INPUT: u32 = (1 << 12) | (1 << 9); // MCU_SEL=GPIO, FUN_IE.
const GPIO_LOW: u32 = 4; // ESP-IDF gpio_types.h.
const GPIO_HIGH: u32 = 5;
const GPIO_TRIGGER: u32 = 1 << 2; // soc/rtc.h, маска WAKEUP_ENA.
const TIMER_TRIGGER: u32 = 1 << 3;
const SLEEP: u32 = 1 << 31;
const ALARM_ENABLE: u32 = 1 << 16;
const DEEP: u32 = 1 << 31; // DIG_PWC.DG_WRAP_PD_EN.
const FASTMEM_FORCE_OFF: u32 = 1 << 15;
const TRM_WDT_KEY: u32 = 0x50d8_3aa1; // TRM рег. 9.37.

// TRM §6.2.4.1 и даташит §4.1.3.3: CPU после сброса XTAL 40 МГц / 2.
// TRM табл. 9.3-1: номинальный RC_SLOW 136 кГц. Независимый расчёт в нс.
const CPU_NS: u64 = 50;
const SLOW_HZ: u64 = 136_000;
const NS: u64 = 1_000_000_000;

fn edge(tick: u64) -> u64 {
    (tick * NS).div_ceil(SLOW_HZ * CPU_NS)
}

// Кодировки RISC-V Unprivileged ISA. Известный PC последнего EBREAK отличает
// успешный опыт от любого проваленного сравнения гостя.
const EBREAK: u32 = 0x0010_0073;

#[derive(Default)]
struct Probe(Vec<u32>);

impl Probe {
    fn pc(&self) -> u32 {
        ROM + size_of_val(self.0.as_slice()) as u32
    }

    fn imm(&mut self, reg: u32, word: u32) {
        let hi = word.wrapping_add(0x800) & !0xfff;
        self.0.extend([
            hi | (reg << 7) | 0x37,
            (word & 0xfff) << 20 | (reg << 15) | (reg << 7) | 0x13,
        ]);
    }

    fn write(&mut self, addr: u32, word: u32) {
        self.imm(5, addr);
        self.imm(6, word);
        self.0.push(0x0062_a023); // sw t1,0(t0).
    }

    fn read(&mut self, addr: u32) {
        self.imm(5, addr);
        self.0.push(0x0002_a383); // lw t2,0(t0).
    }

    fn expect(&mut self, addr: u32, word: u32) {
        self.read(addr);
        self.imm(6, word);
        self.0.extend([0x0063_8463, EBREAK]); // beq t2,t1,+8.
    }

    fn branch_if_equal(&mut self, addr: u32, word: u32) -> usize {
        self.read(addr);
        self.imm(6, word);
        let index = self.0.len();
        self.0.push(0);
        index
    }

    fn point_here(&mut self, branch: usize) {
        let offset = ((self.0.len() - branch) * size_of::<u32>()) as u32;
        assert!(offset < 0x1000 && offset.is_multiple_of(4));
        self.0[branch] = 0x0063_8063
            | ((offset & 0x7e0) << 20)
            | ((offset & 0x1e) << 7)
            | ((offset & 0x800) >> 4);
    }

    fn alarm(&mut self, tick: u32) {
        self.write(ALARM, tick);
        self.write(ALARM_HIGH, ALARM_ENABLE);
    }

    fn image(&self) -> Vec<u8> {
        let mut rom: Vec<_> = self.0.iter().flat_map(|word| word.to_le_bytes()).collect();
        rom.resize(ROM_SIZE, 0);
        rom
    }
}

fn board(rom: &[u8]) -> Box<dyn BoardSim> {
    board_with_profile(rom, None)
}

fn board_with_profile(rom: &[u8], profile: Option<EspResetProfile>) -> Box<dyn BoardSim> {
    // QEMU hw/nvram/esp_efuse.c, esp_efuse_reload_from_blk: с нулевого
    // смещения файла идут слова RDATA. Адреса начала и конца в efuse_reg.h.
    let efuse: Vec<_> = QEMU_EFUSE[..0x17c - 0x2c]
        .chunks_exact(size_of::<u32>())
        .map(|word| u32::from_le_bytes(word.try_into().unwrap()))
        .collect();
    let mut board = new_board(BoardId::Esp32C3);
    let table = esp_image::DEFAULT_PARTITION_TABLE_OFFSET;
    // GPIO_STRAP в QEMU, TRM §5.8: 0x08, SPI boot по GPIO9.
    let straps = [false, false, true];
    match profile {
        Some(profile) => board.load_esp_flash_with_profile(
            FLASH,
            table,
            rom,
            &efuse,
            straps,
            EspBootOptions::default().with_reset_profile(profile),
        ),
        None => board.load_esp_flash(FLASH, table, rom, &efuse, straps),
    }
    .unwrap();
    board
}

fn drive(board: &mut dyn BoardSim, pin: u32, high: bool) {
    board
        .apply(InputEvent {
            cycle: board.cycles(),
            input: BoardInput::Gpio { pin, high },
        })
        .unwrap();
}

fn until(board: &mut dyn BoardSim, cycle: u64) {
    assert_eq!(
        board.run_cycles(cycle - board.cycles()).unwrap().cycle,
        cycle
    );
}

fn finish(board: &mut dyn BoardSim, success: u32) {
    assert_eq!(
        board.run_cycles(128),
        Err(BoardFault::Breakpoint { pc: success })
    );
}

#[test]
fn таймер_останавливает_cpu_и_возобновляет_в_точном_такте_rtc() {
    const ALARM_TICK: u32 = 17;
    let mut probe = Probe::default();
    probe.write(SRAM, 0xca7);
    probe.write(RTC_RAM, 0xba7);
    probe.write(SOURCES, TIMER_TRIGGER << 15);
    probe.write(INT_ENABLE, 1); // RTC_CNTL_SLP_WAKEUP_INT_ENA.
    probe.write(TIMER5, 0);
    probe.alarm(ALARM_TICK);
    probe.write(STATE, SLEEP);
    let after_sleep = probe.pc();
    probe.expect(WAKE_CAUSE, TIMER_TRIGGER);
    probe.expect(INT_RAW, (1 << 10) | 1);
    probe.expect(INT_ST, 1);
    probe.write(INT_CLR, (1 << 10) | 1);
    probe.expect(INT_RAW, 0);
    probe.write(TIME_UPDATE, 1 << 31);
    probe.expect(TIME_LOW, ALARM_TICK);
    probe.expect(SRAM, 0xca7);
    probe.expect(RTC_RAM, 0xba7);
    let success = probe.pc();
    probe.0.push(EBREAK);

    let mut board = board(&probe.image());
    assert_eq!(board.esp_sleep_status().unwrap().mode, EspSleepMode::Modem);
    let wake_cycle = edge(u64::from(ALARM_TICK));
    until(&mut *board, wake_cycle - 1);
    let sleep = board.esp_sleep_status().unwrap();
    assert_eq!(sleep.mode, EspSleepMode::Light);
    assert_eq!(
        sleep.entered_at,
        Some(u64::from(after_sleep - ROM) / size_of::<u32>() as u64)
    );
    assert_eq!(board.esp_boot_position().unwrap().pc, after_sleep);
    assert_eq!(sleep.rtc_slow_ticks, u64::from(ALARM_TICK) - 1);
    let mut branch = board.clone_state();
    // Пройти границу тем же временем в другой единице, через публичный API.
    assert_eq!(
        branch
            .run_until_quarter_ns(u128::from(wake_cycle * CPU_NS * 4))
            .unwrap()
            .unwrap()
            .cycle,
        wake_cycle
    );
    until(&mut *board, wake_cycle);
    let wake = EspWake {
        mode: EspSleepMode::Light,
        cycle: wake_cycle,
        rtc_slow_tick: u64::from(ALARM_TICK),
        timer: true,
        gpio_mask: 0,
    };
    assert_eq!(board.esp_sleep_status().unwrap().last_wake, Some(wake));
    assert_eq!(board.esp_sleep_status(), branch.esp_sleep_status());
    assert_eq!(board.esp_boot_position().unwrap().pc, after_sleep);
    assert_eq!(board.reset_cause(), Some(ResetCause::PowerOn));
    finish(&mut *board, success);
    finish(&mut *branch, success);
}

#[test]
fn gpio_light_sleep_игнорирует_короткий_импульс_и_чужой_вывод() {
    const PIN: u32 = 10;
    let mut probe = Probe::default();
    probe.write(MUX_GPIO + PIN * 4, MUX_GPIO_INPUT);
    probe.write(DIGITAL_GPIO + PIN * 4, (1 << 10) | (GPIO_HIGH << 7));
    probe.write(SOURCES, GPIO_TRIGGER << 15);
    probe.write(TIMER5, 0);
    probe.write(STATE, SLEEP);
    probe.expect(WAKE_CAUSE, GPIO_TRIGGER);
    let success = probe.pc();
    probe.0.push(EBREAK);
    let mut board = board(&probe.image());
    drive(&mut *board, PIN, false);
    until(&mut *board, edge(1));
    drive(&mut *board, PIN, true);
    until(&mut *board, edge(2));
    drive(&mut *board, PIN, false);
    drive(&mut *board, PIN + 1, true);
    until(&mut *board, edge(100));
    assert_eq!(board.esp_sleep_status().unwrap().mode, EspSleepMode::Light);
    assert_eq!(board.esp_sleep_status().unwrap().last_wake, None);
    drive(&mut *board, PIN, true);
    until(&mut *board, edge(103) - 1);
    assert_eq!(board.esp_sleep_status().unwrap().last_wake, None);
    until(&mut *board, edge(103));
    assert_eq!(
        board.esp_sleep_status().unwrap().last_wake,
        Some(EspWake {
            mode: EspSleepMode::Light,
            cycle: edge(103),
            rtc_slow_tick: 103,
            timer: false,
            gpio_mask: 1 << PIN,
        })
    );
    finish(&mut *board, success);
}

#[test]
fn минимальный_сон_удерживает_cpu_до_одновременного_wake() {
    const PIN: u32 = 10;
    const MIN_TICKS: u64 = 0x80; // TRM рег. 9.10, значение сброса MIN_SLP_VAL.
    let mut probe = Probe::default();
    probe.write(MUX_GPIO + PIN * 4, MUX_GPIO_INPUT);
    probe.write(DIGITAL_GPIO + PIN * 4, (1 << 10) | (GPIO_HIGH << 7));
    probe.write(SOURCES, (GPIO_TRIGGER | TIMER_TRIGGER) << 15);
    probe.alarm(4);
    probe.write(STATE, SLEEP);
    probe.expect(WAKE_CAUSE, GPIO_TRIGGER | TIMER_TRIGGER);
    let success = probe.pc();
    probe.0.push(EBREAK);
    let mut board = board(&probe.image());
    drive(&mut *board, PIN, true);
    until(&mut *board, edge(5));
    let mut interrupted = board.clone_state();
    drive(&mut *interrupted, PIN, false);
    assert!(matches!(
        interrupted.run_cycles(1),
        Err(BoardFault::Unsupported(detail)) if detail.contains("минимального времени сна")
    ));
    until(&mut *board, edge(MIN_TICKS) - 1);
    assert_eq!(board.esp_sleep_status().unwrap().last_wake, None);
    until(&mut *board, edge(MIN_TICKS));
    let wake = board.esp_sleep_status().unwrap().last_wake.unwrap();
    assert!(wake.timer);
    assert_eq!(wake.gpio_mask, 1 << PIN);
    assert_eq!(wake.rtc_slow_tick, MIN_TICKS);
    finish(&mut *board, success);
}

#[test]
fn два_deep_wake_с_холодным_стартом_сохраняют_rtc_fast() {
    // TRM §9.6: RTC FAST и STORE переживают Deep-sleep, CPU стартует с ROM.
    const PIN: u32 = 4;
    const ALARM_TICK: u32 = 272_000; // Номинальные 2 с × 136 кГц.
    let mut probe = Probe::default();
    let after_timer = probe.branch_if_equal(RTC_STORE0, 1);
    let after_gpio = probe.branch_if_equal(RTC_STORE0, 2);
    probe.write(SRAM, 0x55aa);
    probe.write(RTC_RAM, 0xca7);
    probe.write(RTC_STORE0, 1);
    probe.write(SOURCES, TIMER_TRIGGER << 15);
    probe.alarm(ALARM_TICK);
    probe.write(POWER, DEEP);
    probe.write(STATE, SLEEP);
    probe.0.push(EBREAK);

    probe.point_here(after_timer);
    probe.expect(RESET_STATE, 0x7005); // TRM рег. 9.12 и reset_reasons.h.
    probe.expect(WAKE_CAUSE, TIMER_TRIGGER);
    probe.expect(RTC_RAM, 0xca7);
    probe.write(RTC_RAM, 0xba7);
    probe.write(RTC_STORE0, 2);
    probe.write(MUX_GPIO + PIN * 4, MUX_GPIO_INPUT);
    probe.write(EXT_WAKEUP, 1 << 31);
    // RTC_CNTL_GPIO_PIN0_WAKEUP_ENABLE, GPIO_PIN0_INT_TYPE, CLK_GATE.
    let gpio_config = (1 << (31 - PIN)) | (GPIO_LOW << (23 - PIN * 3)) | (1 << 7);
    probe.write(RTC_GPIO_WAKE, gpio_config);
    probe.write(SOURCES, GPIO_TRIGGER << 15);
    probe.write(POWER, DEEP);
    probe.write(TIMER5, 0);
    probe.write(STATE, SLEEP);
    probe.0.push(EBREAK);

    probe.point_here(after_gpio);
    probe.expect(RESET_STATE, 0x7005);
    probe.expect(WAKE_CAUSE, GPIO_TRIGGER);
    probe.expect(RTC_GPIO_WAKE, gpio_config | (1 << PIN));
    probe.expect(RTC_RAM, 0xba7);
    let success = probe.pc();
    probe.0.push(EBREAK);

    let mut board = board(&probe.image());
    drive(&mut *board, PIN, true);
    // RTC WDT конфигурации eFuse/загрузчика действует отдельно от wake.
    until(&mut *board, edge(u64::from(ALARM_TICK)) - 1);
    assert_eq!(board.reset_cause(), Some(ResetCause::PowerOn));
    until(&mut *board, edge(u64::from(ALARM_TICK)));
    assert_eq!(board.esp_boot_position().unwrap().pc, ROM);
    assert_eq!(board.esp_boot_position().unwrap().step, 0);
    assert_eq!(board.reset_cause(), Some(ResetCause::DeepSleep));
    assert_eq!(
        board
            .esp_sleep_status()
            .unwrap()
            .last_wake
            .unwrap()
            .rtc_slow_tick,
        u64::from(ALARM_TICK)
    );
    let gpio_start = u64::from(ALARM_TICK) + 10;
    until(&mut *board, edge(gpio_start));
    assert_eq!(board.esp_sleep_status().unwrap().mode, EspSleepMode::Deep);
    assert_eq!(
        board
            .esp_sleep_status()
            .unwrap()
            .last_wake
            .unwrap()
            .gpio_mask,
        0
    );
    drive(&mut *board, PIN, false);
    until(&mut *board, edge(gpio_start + 3));
    assert_eq!(board.esp_boot_position().unwrap().step, 0);
    assert_eq!(
        board.esp_sleep_status().unwrap().last_wake,
        Some(EspWake {
            mode: EspSleepMode::Deep,
            cycle: edge(gpio_start + 3),
            rtc_slow_tick: gpio_start + 3,
            timer: false,
            gpio_mask: 1 << PIN,
        })
    );
    let resets: Vec<_> = board
        .take_events()
        .into_iter()
        .filter_map(|event| {
            (event.output == BoardOutput::Reset(ResetCause::DeepSleep)).then_some(event.cycle)
        })
        .collect();
    assert_eq!(resets, [edge(u64::from(ALARM_TICK)), edge(gpio_start + 3)]);
    eprintln!(
        "RTC SLOW: таймер {ALARM_TICK} и GPIO {}, такты платы {resets:?}",
        gpio_start + 3
    );
    finish(&mut *board, success);
}

#[test]
fn deep_sleep_теряет_sram_и_учитывает_запрет_retention() {
    for (power, lost_addr) in [(DEEP, SRAM), (DEEP | FASTMEM_FORCE_OFF, RTC_RAM)] {
        let mut probe = Probe::default();
        let after_sleep = probe.branch_if_equal(RTC_STORE0, 1);
        probe.write(SRAM, 7);
        probe.write(RTC_RAM, 8);
        probe.write(RTC_STORE0, 1);
        probe.write(SOURCES, TIMER_TRIGGER << 15);
        probe.alarm(5);
        probe.write(POWER, power);
        probe.write(TIMER5, 0);
        probe.write(STATE, SLEEP);
        probe.0.push(EBREAK);
        probe.point_here(after_sleep);
        probe.read(lost_addr);
        probe.0.push(EBREAK);
        let mut board = board(&probe.image());
        let error = board.run_cycles(edge(6)).unwrap_err();
        assert!(
            matches!(error, BoardFault::Unsupported(ref detail)
            if detail.contains(&format!("{lost_addr:#010x}"))),
            "{error}"
        );
        assert_eq!(board.reset_cause(), Some(ResetCause::DeepSleep));
    }
}

#[test]
fn активный_gpio_отклоняет_сон_а_не_сбрасывает_cpu() {
    const PIN: u32 = 10;
    let mut probe = Probe::default();
    probe.write(MUX_GPIO + PIN * 4, MUX_GPIO_INPUT);
    probe.write(DIGITAL_GPIO + PIN * 4, (1 << 10) | (GPIO_HIGH << 7));
    probe.write(SOURCES, GPIO_TRIGGER << 15);
    probe.write(REJECT_CONF, (1 << 30) | (GPIO_TRIGGER << 12));
    probe.write(STATE, SLEEP);
    probe.expect(REJECT_CAUSE, GPIO_TRIGGER);
    probe.expect(INT_RAW, 1 << 1);
    probe.write(STATE, 1 << 1);
    probe.expect(REJECT_CAUSE, 0);
    probe.write(INT_CLR, 1 << 1);
    probe.expect(INT_RAW, 0);
    let success = probe.pc();
    probe.0.push(EBREAK);
    let mut board = board(&probe.image());
    drive(&mut *board, PIN, true);
    finish(&mut *board, success);
    assert_eq!(board.reset_cause(), Some(ResetCause::PowerOn));
}

#[test]
fn rtc_сторож_в_light_sleep_уважает_pause_in_sleep() {
    for paused in [true, false] {
        let mut probe = Probe::default();
        let after_reset = probe.branch_if_equal(RTC_STORE0, 1);
        probe.write(RTC_STORE0, 1);
        probe.write(WDT_WPROTECT, TRM_WDT_KEY);
        probe.write(WDT_STAGE0, 4);
        // WDT_EN, WDT_STG0 = Reset CPU, PAUSE_IN_SLEEP, PROCPU_RESET_EN.
        probe.write(
            WDT_CONFIG0,
            (1 << 31) | (2 << 28) | (1 << 11) | (u32::from(paused) << 9),
        );
        probe.write(SOURCES, 0);
        probe.write(STATE, SLEEP);
        probe.0.push(EBREAK);
        probe.point_here(after_reset);
        let success = probe.pc();
        probe.0.push(EBREAK);
        let mut board = board(&probe.image());
        if paused {
            until(&mut *board, edge(16));
            assert_eq!(board.reset_cause(), Some(ResetCause::PowerOn));
            assert_eq!(board.esp_sleep_status().unwrap().mode, EspSleepMode::Light);
        } else {
            assert_eq!(
                board.run_cycles(edge(16)),
                Err(BoardFault::Breakpoint { pc: success })
            );
            assert_eq!(board.reset_cause(), Some(ResetCause::Watchdog));
        }
    }
}

#[test]
fn плавающий_gpio_оставляет_пробуждение_неопределённым() {
    const PIN: u32 = 10;
    let mut probe = Probe::default();
    probe.write(MUX_GPIO + PIN * 4, MUX_GPIO_INPUT);
    probe.write(DIGITAL_GPIO + PIN * 4, (1 << 10) | (GPIO_LOW << 7));
    probe.write(SOURCES, GPIO_TRIGGER << 15);
    probe.write(STATE, SLEEP);
    probe.0.push(EBREAK);
    let mut board = board(&probe.image());
    assert!(matches!(
        board.run_cycles(edge(200)),
        Err(BoardFault::Unsupported(detail)) if detail.contains("уровень пробуждения не определён")
    ));
}

#[test]
fn неизвестный_источник_вывод_или_crc_даёт_явную_ошибку() {
    for (register, value, message) in [
        (SOURCES, 0x20 << 15, "источник сна"),
        (SOURCES, GPIO_TRIGGER << 15, "без настроенного вывода"),
        (CRC_CONFIG, CRC_RESET | CRC_START, "CRC RTC FAST"),
    ] {
        let mut probe = Probe::default();
        probe.write(register, value);
        probe.write(STATE, SLEEP);
        probe.0.push(EBREAK);
        let mut board = board(&probe.image());
        let error = board.run_cycles(128).unwrap_err();
        assert!(
            matches!(&error, BoardFault::Unsupported(detail)
            if detail.contains(message)),
            "{error}"
        );
        assert_eq!(board.reset_cause(), Some(ResetCause::PowerOn));
    }
}

#[test]
fn crc_rtc_fast_поля_и_права_trm() {
    let mut probe = Probe::default();
    // TRM рег. 16.11: LEN[30:20]=0x7ff и ADDR[19:9]=0 при сбросе.
    probe.expect(CRC_CONFIG, CRC_RESET);
    probe.expect(CRC_RESULT, 0);
    // Проверяем независимые поля LEN=2 и ADDR=1, а также запись бита RO.
    let config = (2 << 20) | (1 << 9);
    probe.write(CRC_CONFIG, config | CRC_FINISH);
    probe.expect(CRC_CONFIG, config);
    probe.expect(CRC_RESULT, 0);
    let success = probe.pc();
    probe.0.push(EBREAK);
    finish(&mut *board(&probe.image()), success);

    let mut probe = Probe::default();
    probe.write(CRC_RESULT, 1);
    assert_eq!(
        board(&probe.image()).run_cycles(128),
        Err(BoardFault::WriteToReadOnly {
            pc: probe.pc() - size_of::<u32>() as u32,
            addr: CRC_RESULT,
        })
    );
}

#[test]
fn профиль_qemu_crc_не_подделывает_завершение_и_результат() {
    let profile = EspResetProfile::parse(CRC_PROFILE).unwrap();
    let [entry] = profile.entries() else {
        panic!("ожидалась одна запись наблюдения CRC")
    };
    assert_eq!(
        (entry.address, entry.kind),
        (CRC_CONFIG, EspResetValueKind::Qemu)
    );
    let read = format!("read={:#x}", entry.value);
    let polls = format!(
        "write={:#x} write={:#x} {read} {read}",
        CRC_RESET,
        CRC_RESET | CRC_START
    );
    assert_eq!(
        QEMU_TRACE.matches(&polls).count(),
        3,
        "echo, corrupt, silent"
    );
    assert!(
        EspResetProfile::parse(&CRC_PROFILE.replace("kind = \"qemu\"", "kind = \"doc\"")).is_err()
    );
    assert!(
        EspResetProfile::parse(&CRC_PROFILE.replace("value = 0", "value = 0x80000000")).is_err()
    );

    for (addr, expect_error) in [(CRC_RESULT, "результата"), (STATE, "до CRC_FINISH")] {
        let mut probe = Probe::default();
        probe.write(POWER, DEEP);
        probe.write(SOURCES, TIMER_TRIGGER << 15);
        probe.alarm(200);
        probe.write(CRC_CONFIG, CRC_RESET | CRC_START);
        probe.expect(CRC_CONFIG, 0);
        probe.expect(CRC_CONFIG, 0);
        if addr == CRC_RESULT {
            probe.read(addr);
        } else {
            probe.write(addr, SLEEP);
        }
        let failure_pc = probe.pc() - size_of::<u32>() as u32;
        let mut board = board_with_profile(&probe.image(), Some(profile.clone()));
        for _ in 0..128 {
            if board.esp_boot_position().unwrap().pc == failure_pc {
                break;
            }
            board.run_cycles(1).unwrap();
        }
        assert_eq!(board.esp_boot_position().unwrap().pc, failure_pc);
        let status = board.esp_sleep_status().unwrap();
        assert_eq!((status.entered_at, status.last_wake), (None, None));
        assert!(
            matches!(board.run_cycles(1), Err(BoardFault::Unsupported(detail))
            if detail.contains(expect_error))
        );
        assert_eq!(board.esp_boot_position().unwrap().pc, failure_pc);
        let [used] = board.esp_reset_value_uses() else {
            panic!("нет квитанции чтений CRC")
        };
        assert_eq!(
            (&used.entry, used.reads, used.read_addresses[&CRC_CONFIG]),
            (entry, 2, 2)
        );
        assert!(
            !board
                .take_events()
                .iter()
                .any(|event| { event.output == BoardOutput::Reset(ResetCause::DeepSleep) })
        );
        assert!(board.capabilities().blocks_hardware_proof);
    }
}

#[test]
#[ignore = "требуется дамп ROM из установленного QEMU Espressif в ESP_SLEEP_QEMU_ROM"]
fn штатный_образ_2_21_показывает_барьер_сквозного_прогона() {
    let rom = std::fs::read(std::env::var_os("ESP_SLEEP_QEMU_ROM").expect("нужен ROM QEMU"))
        .expect("чтение ROM");
    let mut board = board(&rom);
    let failure = board.run_cycles(1_000_000).unwrap_err(); // Бюджет диагностического прогона.
    // ROM Espressif rev. 3, esp32c3_rev3_rom.elf: UART_STATUS_REG.
    // Трасса QEMU 9.2.2 даёт тот же PC на инструкции №391182.
    let position = EspBootPosition {
        step: 6195,
        pc: 0x4004_c0d4,
    };
    assert_eq!(board.esp_boot_position(), Some(position));
    eprintln!(
        "BoardSim: шаг {}, PC {:#010x}, {failure}",
        position.step + 1,
        position.pc
    );
    assert!(
        !board
            .take_events()
            .iter()
            .any(|e| e.output == BoardOutput::Reset(ResetCause::DeepSleep))
    );
    assert_eq!(
        failure,
        BoardFault::UnsupportedRegister {
            pc: position.pc,
            addr: 0x6000_001c,
            size: 4,
            write: false,
            block: "UART: не заданы уровни линий UART_STATUS",
        }
    );
    assert_eq!(board.run_cycles(1), Err(failure));
}
