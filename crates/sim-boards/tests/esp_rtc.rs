use sim_boards::{
    BoardFault, BoardId, BoardInput, BoardOutput, EspRtc, EspRtcReset, InputEvent, ResetCause,
    new_board,
};

// TRM v1.4 §§9.7, 12.2; ESP-IDF soc/esp32c3/register/soc/rtc_cntl_reg.h.
const RTC: u32 = 0x6000_8000;
const OPTIONS0: u32 = RTC;
const TIME_UPDATE: u32 = RTC + 0x0c;
const TIME_LOW: u32 = RTC + 0x10;
const TIME_HIGH: u32 = RTC + 0x14;
const RESET_STATE: u32 = RTC + 0x38;
const INT_ENA: u32 = RTC + 0x40;
const INT_RAW: u32 = RTC + 0x44;
const INT_ST: u32 = RTC + 0x48;
const INT_CLR: u32 = RTC + 0x4c;
const STORE0: u32 = RTC + 0x50;
const DIG_PWC: u32 = RTC + 0x88;
const DIG_ISO: u32 = RTC + 0x8c;
const WDTCONFIG0: u32 = RTC + 0x90;
const WDTCONFIG1: u32 = RTC + 0x94;
const WDTCONFIG2: u32 = RTC + 0x98;
const WDTCONFIG3: u32 = RTC + 0x9c;
const WDTCONFIG4: u32 = RTC + 0xa0;
const WDTFEED: u32 = RTC + 0xa4;
const WDTWPROTECT: u32 = RTC + 0xa8;
const SWD_CONF: u32 = RTC + 0xac;
const SWD_WPROTECT: u32 = RTC + 0xb0;
const STORE6: u32 = RTC + 0xc0;
const INT_ENA_SET: u32 = RTC + 0x100;
const INT_ENA_CLEAR: u32 = RTC + 0x104;

const WDT_KEY: u32 = 0x50d8_3aa1;
const WDT_EN: u32 = 1 << 31;
const WDT_FLASHBOOT: u32 = 1 << 12;
const PROCPU_RESET_EN: u32 = 1 << 11;
const PAUSE_IN_SLEEP: u32 = 1 << 9;
const WDT_IRQ: u32 = 1 << 3;
const FEED: u32 = 1 << 31;
const SWD_KEY: u32 = 0x8f1d_312a;
const SWD_AUTO_FEED: u32 = 1 << 31;
const CLEAR_RESET_FLAG: u32 = 1 << 16;
const RESET_FLAG: u32 = 1 << 14;
const SW_CPU_RST: u32 = 1 << 5;
const SW_SYS_RST: u32 = 1 << 31;

// TRM §3.3.2, начало SRAM1 (IBUS) и RTC FAST.
const PC: u32 = 0x4038_0000;
const RTC_FAST: u32 = 0x5000_0000;
const WORD: u8 = size_of::<u32>() as u8;

fn read(rtc: &mut EspRtc, addr: u32) -> u32 {
    rtc.access(PC, addr, WORD, None).unwrap().unwrap()
}

fn write(rtc: &mut EspRtc, addr: u32, value: u32) {
    assert_eq!(rtc.access(PC, addr, WORD, Some(value)), Ok(Some(0)));
}

#[test]
fn причины_сброса_и_удержание_store() {
    let mut rtc = EspRtc::default();
    // Независимый эталон: QEMU Espressif 9.2.2, штатный hello_world ESP-IDF 6.1.1.
    // xp /1wx 0x60008038 показал 0x00000001 и 0x0000000c после esp_restart;
    // QEMU реализует код причины, а не остальные поля слова.
    assert_eq!(read(&mut rtc, RESET_STATE) & 0x3f, 0x01);
    let status = read(&mut rtc, RESET_STATE);
    write(&mut rtc, RESET_STATE, status | CLEAR_RESET_FLAG);
    assert_eq!(read(&mut rtc, RESET_STATE), status & !RESET_FLAG);

    // RTC_RESET_CAUSE_REG указывает на STORE6. Значение стенда отлично от нуля.
    let marker = 0x8007_0007;
    write(&mut rtc, STORE6, marker);
    write(&mut rtc, OPTIONS0, SW_CPU_RST);
    assert_eq!(rtc.advance_slow(2, false), 0);
    assert_eq!(rtc.pending_reset(), Some(EspRtcReset::SoftwareCpu));
    assert_eq!(rtc.reset_reason(), Some(EspRtcReset::PowerOn));
    rtc.reset(EspRtcReset::SoftwareCpu);
    assert_eq!(read(&mut rtc, RESET_STATE) & 0x3f, 0x0c);
    assert_eq!(read(&mut rtc, OPTIONS0) & SW_CPU_RST, 0);
    assert_eq!(read(&mut rtc, STORE6), marker);
    assert!(EspRtcReset::SoftwareCpu.retains_rtc_fast());

    write(&mut rtc, OPTIONS0, SW_SYS_RST);
    assert_eq!(rtc.pending_reset(), Some(EspRtcReset::SoftwareCore));
    rtc.reset(EspRtcReset::SoftwareCore);
    assert_eq!(read(&mut rtc, RESET_STATE) & 0x3f, 0x03); // TRM табл. 6.1-1.
    assert_eq!(read(&mut rtc, STORE6), marker);

    rtc.reset(EspRtcReset::PowerOn);
    assert_eq!(read(&mut rtc, RESET_STATE) & 0x3f, 0x01);
    assert_eq!(read(&mut rtc, STORE6), 0);
    assert!(!EspRtcReset::PowerOn.retains_rtc_fast());
}

#[test]
fn rwdt_два_порога_ключ_маска_и_feed() {
    let mut rtc = EspRtc::new(2).unwrap();
    // rwdt_ll_config_stage(): для запроса 24 тактов при eFuse=2 пишет 24>>3=3.
    // TRM 9.31: stage0=1 (IRQ), stage1=3 (Core Reset).
    let config = WDT_EN | (1 << 28) | (3 << 25);
    write(&mut rtc, WDTCONFIG0, config);
    assert_eq!(read(&mut rtc, WDTCONFIG0) & WDT_EN, 0); // TRM §12.2.2.3.
    write(&mut rtc, WDTWPROTECT, WDT_KEY);
    write(&mut rtc, WDTCONFIG1, 3);
    write(&mut rtc, WDTCONFIG2, 5);
    write(&mut rtc, WDTCONFIG0, config);
    write(&mut rtc, WDTWPROTECT, 0);

    assert_eq!(rtc.advance_slow(23, false), 23);
    assert_eq!(read(&mut rtc, INT_RAW), 0);
    write(&mut rtc, WDTFEED, FEED); // заблокирован.
    assert_eq!(rtc.advance_slow(1, false), 1);
    assert_eq!(read(&mut rtc, INT_RAW), WDT_IRQ);
    assert_eq!(read(&mut rtc, INT_ST), 0);
    assert!(!rtc.interrupt_pending());
    write(&mut rtc, INT_ENA_SET, WDT_IRQ);
    assert!(rtc.interrupt_pending());
    assert_eq!(read(&mut rtc, INT_ENA), WDT_IRQ);
    assert_eq!(read(&mut rtc, INT_ST), WDT_IRQ);
    write(&mut rtc, INT_ENA_CLEAR, WDT_IRQ);
    assert!(!rtc.interrupt_pending());
    assert_eq!(read(&mut rtc, INT_RAW), WDT_IRQ);
    write(&mut rtc, INT_CLR, WDT_IRQ);
    assert_eq!(read(&mut rtc, INT_RAW), 0);

    write(&mut rtc, WDTWPROTECT, WDT_KEY);
    write(&mut rtc, WDTFEED, FEED);
    write(&mut rtc, STORE0, 57); // метка стенда.
    assert_eq!(rtc.advance_slow(28, false), 28);
    assert_eq!(rtc.pending_reset(), None);
    let mut snapshot = rtc.clone();
    for model in [&mut rtc, &mut snapshot] {
        assert_eq!(model.advance_slow(10, false), 1);
        assert_eq!(model.pending_reset(), Some(EspRtcReset::WatchdogCore));
        model.reset(EspRtcReset::WatchdogCore);
        assert_eq!(read(model, RESET_STATE) & 0x3f, 0x09); // TRM табл. 6.1-1.
        assert_eq!(read(model, STORE0), 57);
    }
}

#[test]
fn rwdt_cpu_system_flashboot_и_pause() {
    let mut rtc = EspRtc::new(0).unwrap();
    write(&mut rtc, WDTWPROTECT, WDT_KEY);
    write(&mut rtc, WDTCONFIG1, 2);
    // TRM рег. 9.31: действие 2 разрешает сброс CPU только с PROCPU_RESET_EN.
    write(&mut rtc, WDTCONFIG0, WDT_EN | (2 << 28) | PAUSE_IN_SLEEP);
    assert_eq!(rtc.advance_slow(4, false), 4);
    assert_eq!(rtc.pending_reset(), None);
    write(&mut rtc, WDTFEED, FEED);
    write(
        &mut rtc,
        WDTCONFIG0,
        WDT_EN | (2 << 28) | PAUSE_IN_SLEEP | PROCPU_RESET_EN,
    );
    assert_eq!(rtc.advance_slow(10, true), 10);
    assert_eq!(rtc.advance_slow(3, false), 3);
    assert_eq!(rtc.pending_reset(), None);
    assert_eq!(rtc.advance_slow(1, false), 1);
    assert_eq!(rtc.pending_reset(), Some(EspRtcReset::WatchdogCpu));
    rtc.reset(EspRtcReset::WatchdogCpu);
    assert_eq!(read(&mut rtc, RESET_STATE) & 0x3f, 0x0d); // TRM табл. 6.1-1.

    // TRM §12.2.2.4: Flash Boot Mode действует и при WDT_EN=0.
    write(&mut rtc, WDTCONFIG0, WDT_FLASHBOOT);
    write(&mut rtc, STORE0, 57);
    rtc.enter_flash_boot().unwrap();
    assert_eq!(rtc.advance_slow(3, false), 3);
    assert_eq!(rtc.pending_reset(), None);
    assert_eq!(rtc.advance_slow(3, false), 1);
    assert_eq!(rtc.pending_reset(), Some(EspRtcReset::WatchdogSystem));
    rtc.reset(EspRtcReset::WatchdogSystem);
    assert_eq!(read(&mut rtc, RESET_STATE) & 0x3f, 0x10); // TRM табл. 6.1-1.
    assert_eq!(read(&mut rtc, STORE0), 0);
    assert_eq!(read(&mut rtc, WDTWPROTECT), 0);
    assert!(EspRtcReset::WatchdogSystem.retains_rtc_fast());

    rtc.enter_flash_boot().unwrap();
    write(&mut rtc, WDTWPROTECT, WDT_KEY);
    write(&mut rtc, WDTCONFIG0, 0);
    assert_eq!(rtc.advance_slow(400_001, false), 400_001); // выше 200_000 << 1.
    assert_eq!(rtc.pending_reset(), None);
}

#[test]
fn rwdt_проходит_четыре_ступени_и_зацикливается() {
    let mut rtc = EspRtc::new(1).unwrap();
    write(&mut rtc, WDTWPROTECT, WDT_KEY);
    // TRM §12.2.2.2: 1 << (1+1), 5, 7, 11. Полный оборот равен 27 тактам.
    for (reg, hold) in [
        (WDTCONFIG1, 1),
        (WDTCONFIG2, 5),
        (WDTCONFIG3, 7),
        (WDTCONFIG4, 11),
    ] {
        write(&mut rtc, reg, hold);
    }
    // TRM регистр 9.31: только третья ступень выдаёт IRQ.
    write(&mut rtc, WDTCONFIG0, WDT_EN | (1 << 22));
    write(&mut rtc, INT_ENA, WDT_IRQ);
    assert_eq!(rtc.advance_slow(15, false), 15);
    assert!(!rtc.interrupt_pending());
    assert_eq!(rtc.advance_slow(1, false), 1);
    assert!(rtc.interrupt_pending());
    write(&mut rtc, INT_CLR, WDT_IRQ);
    let mut one_tick = rtc.clone();
    assert_eq!(rtc.advance_slow(27, false), 27);
    for cycle in 1..=27 {
        assert_eq!(one_tick.advance_slow(1, false), 1);
        assert_eq!(one_tick.interrupt_pending(), cycle == 27);
    }
    assert_eq!(rtc.interrupt_pending(), one_tick.interrupt_pending());
    assert_eq!(rtc.pending_reset(), None);

    // Из текущей точки до конца четвёртой ступени остаётся 11 тактов.
    write(&mut rtc, WDTCONFIG0, WDT_EN | (1 << 22) | (4 << 19));
    assert_eq!(rtc.advance_slow(10, false), 10);
    assert_eq!(rtc.pending_reset(), None);
    assert_eq!(rtc.advance_slow(2, false), 1);
    assert_eq!(rtc.pending_reset(), Some(EspRtcReset::WatchdogSystem));
}

#[test]
fn неподдержанное_mmio_и_невалидные_пороги_останавливают_модель_без_изменения() {
    assert!(matches!(EspRtc::new(4), Err(BoardFault::Misconfigured(_))));
    let mut rtc = EspRtc::default();
    assert!(matches!(
        rtc.enter_flash_boot(),
        Err(BoardFault::Unsupported(_))
    ));
    write(&mut rtc, WDTWPROTECT, WDT_KEY);
    let reset_config = read(&mut rtc, WDTCONFIG0);
    for (word, message) in [
        (WDT_EN, "EFUSE_WDT_DELAY_SEL"),
        (5 << 28, "код действия"),
        (1 << 8, "Chip Reset"), // IDF rtc_cntl_reg.h: WDT_CHIP_RESET_EN.
    ] {
        assert!(matches!(rtc.access(PC, WDTCONFIG0, WORD, Some(word)),
            Err(BoardFault::Unsupported(reason)) if reason.contains(message)));
        assert_eq!(read(&mut rtc, WDTCONFIG0), reset_config);
    }
    let mut rtc = EspRtc::new(0).unwrap();
    write(&mut rtc, WDTWPROTECT, WDT_KEY);
    write(&mut rtc, WDTCONFIG1, 0);
    assert!(matches!(rtc.access(PC, WDTCONFIG0, WORD, Some(WDT_EN)),
        Err(BoardFault::Unsupported(reason)) if reason.contains("нулевой")));
    assert_eq!(read(&mut rtc, WDTCONFIG0), reset_config);
    assert_eq!(rtc.access(PC, RTC - 4, WORD, None), Ok(None));
    // TRM §9.7: 0x01f8 не соответствует описанному регистру.
    for (addr, size, value) in [
        (RTC + 0x1f8, WORD, None),
        (RESET_STATE, 2, None),
        (INT_RAW, WORD, Some(WDT_IRQ)),
    ] {
        assert_eq!(
            rtc.access(PC, addr, size, value),
            Err(BoardFault::UnsupportedRegister {
                pc: PC,
                addr,
                size,
                write: value.is_some(),
                block: "RTC_CNTL ESP32-C3",
            })
        );
    }
}

#[test]
fn домены_удержание_и_swd_auto_feed() {
    let mut rtc = EspRtc::default();
    let initial = rtc.domains();
    assert!(
        initial.cpu && initial.digital && initial.peripherals && initial.wifi && initial.bluetooth
    );
    assert_eq!(initial.rtc_fast_sleep_retention, Some(true));
    let initial_pwc = read(&mut rtc, DIG_PWC);
    let initial_iso = read(&mut rtc, DIG_ISO);
    // TRM регистры 9.29/9.30: CPU_DOWN[21]; FASTMEM_FORCE_LPU/LPD[16:15].
    let force_cpu_off_and_drop_fast =
        (initial_pwc & !((1 << 22) | (1 << 16))) | (1 << 21) | (1 << 15);
    write(&mut rtc, DIG_PWC, force_cpu_off_and_drop_fast);
    assert!(!rtc.domains().cpu);
    assert!(rtc.domains().digital && rtc.domains().peripherals);
    assert_eq!(rtc.domains().rtc_fast_sleep_retention, Some(false));
    write(&mut rtc, DIG_PWC, force_cpu_off_and_drop_fast & !(1 << 15));
    assert_eq!(rtc.domains().rtc_fast_sleep_retention, None);
    assert!(matches!(
        rtc.access(PC, DIG_PWC, WORD, Some(initial_pwc | (1 << 21))),
        Err(BoardFault::Unsupported(_))
    ));
    assert!(matches!(
        rtc.access(PC, DIG_ISO, WORD, Some(initial_iso | (1 << 26))),
        Err(BoardFault::Unsupported(_))
    ));
    assert_eq!(read(&mut rtc, DIG_ISO), initial_iso);

    let super_watchdog = read(&mut rtc, SWD_CONF);
    write(&mut rtc, SWD_CONF, super_watchdog | SWD_AUTO_FEED);
    assert_eq!(read(&mut rtc, SWD_CONF), super_watchdog);
    write(&mut rtc, SWD_WPROTECT, SWD_KEY);
    write(&mut rtc, SWD_CONF, super_watchdog | SWD_AUTO_FEED);
    assert_eq!(read(&mut rtc, SWD_CONF), super_watchdog | SWD_AUTO_FEED);
}

#[test]
fn снимок_48_битного_счётчика_переживает_cpu_reset() {
    let mut rtc = EspRtc::default();
    // TRM §9.3.3 и регистры 9.4–9.6: выход обновляется по TIME_UPDATE[31].
    let ticks = (1_u64 << 32) + 7;
    assert_eq!(rtc.advance_slow(ticks, false), ticks);
    assert_eq!(read(&mut rtc, TIME_HIGH), 0);
    write(&mut rtc, TIME_UPDATE, 1 << 31);
    assert_eq!(read(&mut rtc, TIME_HIGH), 1);
    assert_eq!(read(&mut rtc, TIME_LOW), 7);
    rtc.advance_slow(5, false);
    rtc.reset(EspRtcReset::SoftwareCpu);
    assert_eq!(read(&mut rtc, TIME_LOW), 7);
    write(&mut rtc, TIME_UPDATE, 1 << 31);
    assert_eq!(read(&mut rtc, TIME_LOW), 12);
    rtc.advance_slow((1_u64 << 48) - ticks - 5 + 3, false);
    write(&mut rtc, TIME_UPDATE, 1 << 31);
    assert_eq!(read(&mut rtc, TIME_HIGH), 0);
    assert_eq!(read(&mut rtc, TIME_LOW), 3);
}

// ELF32, PT_LOAD: System V ABI, ELF Header и Program Header.
// RV32I код расположен в SRAM1 через прежний вход BoardSim::load.
fn program(words: &[u32]) -> Vec<u8> {
    const ELF_HEADER: usize = 52;
    const PH_HEADER: usize = 32;
    let payload_offset = ELF_HEADER + PH_HEADER;
    let size = size_of_val(words) as u32;
    let mut elf = vec![0; payload_offset];
    elf[..7].copy_from_slice(b"\x7fELF\x01\x01\x01");
    elf[16..18].copy_from_slice(&2_u16.to_le_bytes()); // ET_EXEC.
    elf[18..20].copy_from_slice(&243_u16.to_le_bytes()); // EM_RISCV, RISC-V psABI.
    for (at, value) in [
        (20, 1),
        (24, PC),
        (28, ELF_HEADER as u32),
        (52, 1),
        (56, payload_offset as u32),
        (60, PC),
        (64, PC),
        (68, size),
        (72, size),
        (76, 5),
        (80, 4),
    ] {
        elf[at..at + size_of::<u32>()].copy_from_slice(&value.to_le_bytes());
    }
    for (at, value) in [(40, ELF_HEADER as u16), (42, PH_HEADER as u16), (44, 1_u16)] {
        elf[at..at + size_of::<u16>()].copy_from_slice(&value.to_le_bytes());
    }
    elf.extend(words.iter().flat_map(|word| word.to_le_bytes()));
    elf
}

#[test]
fn boardsim_cpu_reset_оставляет_rtc_fast_store_и_вывод_до_снятия_питания() {
    // RISC-V ISA RV32I. На первом старте выставляем GPIO0, пишем 57 в RTC FAST/STORE6,
    // запрашиваем CPU reset. На втором проверяем точные значения и идём к EBREAK успеха.
    // GPIO_OUT_W1TS/ENABLE_W1TS — TRM рег. 5.3 и 5.6; IO_MUX_GPIO0 — регистр 5.26.
    let words = [
        RTC | 0x2b7,      // lui t0,RTC
        0x0382_a303,      // lw t1,RESET_STATE(t0)
        0x03f3_7313,      // andi t1,t1,0x3f
        0x00c0_0393,      // li t2,0x0c
        0x0473_0663,      // beq t1,t2,ветка повторного старта
        RTC_FAST | 0x437, // lui s0,RTC_FAST
        0x0004_2303,      // lw t1,0(s0)
        0x0403_1e63,      // bnez t1,ошибка (холодный legacy ELF читает ноль)
        0x0c02_a303,      // lw t1,STORE6(t0)
        0x0403_1a63,      // bnez t1,ошибка
        0x0390_0393,      // li t2,57
        0x0074_2023,      // sw t2,0(s0)
        0x0c72_a023,      // sw t2,STORE6(t0)
        0x6000_94b7,      // lui s1,0x60009 (IO_MUX)
        0x0000_1e37,      // lui t3,1 (MCU_SEL=1)
        0x01c4_a223,      // sw t3,4(s1)
        0x6000_44b7,      // lui s1,0x60004 (GPIO)
        0x0010_0393,      // li t2,1
        0x0074_a423,      // sw t2,8(s1) GPIO_OUT_W1TS
        0x0274_a223,      // sw t2,36(s1) GPIO_ENABLE_W1TS
        0x0200_0393,      // li t2,SW_CPU_RST
        0x0072_a023,      // sw t2,OPTIONS0(t0)
        0x0010_0073,      // ebreak: до reset доходить нельзя
        RTC_FAST | 0x437, // lui s0,RTC_FAST (ветка повторного старта)
        0x0390_0393,      // li t2,57
        0x0004_2303,      // lw t1,0(s0)
        0x0073_1863,      // bne t1,t2,ошибка
        0x0c02_a303,      // lw t1,STORE6(t0)
        0x0073_1463,      // bne t1,t2,ошибка
        0x0010_0073,      // ebreak: успех
        0x0010_0073,      // ebreak: ошибка
    ];
    let mut board = new_board(BoardId::Esp32C3);
    board.load(&program(&words)).unwrap();
    let cold_cycles = 22; // От LUI до единственной записи команды reset включительно.
    board.run_cycles(cold_cycles).unwrap();
    assert_eq!(board.reset_cause(), Some(ResetCause::Software));
    assert_eq!(board.pin_output(0), Some(true));
    assert!(
        board
            .take_events()
            .iter()
            .any(|event| event.cycle == cold_cycles
                && event.output == BoardOutput::Reset(ResetCause::Software))
    );
    let success = BoardFault::Breakpoint {
        pc: PC + 29 * u32::from(WORD),
    };
    assert_eq!(board.run_cycles(64), Err(success.clone())); // бюджет выше числа команд.
    for on in [false, true] {
        board
            .apply(InputEvent {
                cycle: board.cycles(),
                input: BoardInput::Power { on },
            })
            .unwrap();
    }
    assert_eq!(board.reset_cause(), Some(ResetCause::PowerOn));
    assert_eq!(board.pin_output(0), None);
    assert_eq!(board.run_cycles(64), Err(success));
}

#[test]
fn boardsim_останавливается_при_запуске_rwdt_без_efuse() {
    // RV32I и TRM регистры 9.31/9.37: снять защиту, установить WDT_EN.
    let words = [
        RTC | 0x2b7, // lui t0,RTC
        0x50d8_4337, // lui t1,0x50d84
        0xaa13_0313, // addi t1,t1,-1375 -> WDT_KEY
        0x0a62_a423, // sw t1,0xa8(t0)
        0x8000_0337, // lui t1,0x80000 -> WDT_EN
        0x0862_a823, // sw t1,0x90(t0)
        0x0010_0073, // ebreak: RWDT без eFuse сюда доходить не должен
    ];
    let mut board = new_board(BoardId::Esp32C3);
    board.load(&program(&words)).unwrap();
    assert!(matches!(board.run_cycles(words.len() as u64),
        Err(BoardFault::Unsupported(reason)) if reason.contains("EFUSE_WDT_DELAY_SEL")));
    assert_eq!(board.cycles(), 5); // Число инструкций до записи WDTCONFIG0.
    assert_eq!(board.watchdog_resets(), 0);
}
