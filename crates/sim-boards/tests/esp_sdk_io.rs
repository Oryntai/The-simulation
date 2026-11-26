//! GPIO Matrix, LEDC и SPI2 через инструкции гостя и штатный путь BoardSim.

use sim_boards::{
    BoardFault, BoardId, BoardInput, BoardOutput, BoardSim, EspBootPosition, InputEvent, SpiMode,
    TransferStatus, esp_image, new_board,
};

// TRM §3.3.2, §5.14, §24, §27 и ESP-IDF soc/esp32c3/register/soc/reg_base.h.
const ROM: u32 = 0x4000_0000;
const ROM_LEN: usize = 384 * 1024;
const GPIO: u32 = 0x6000_4000;
const MUX: u32 = 0x6000_9000;
const SYSTEM: u32 = 0x600c_0000;
const LEDC: u32 = 0x6001_9000;
const SPI: u32 = 0x6002_4000;
const CLK_EN: u32 = SYSTEM + 0x10; // SYSTEM_PERIP_CLK_EN0_REG.
const CLK_RESET: u32 = 0xf9c1_e06f; // TRM рег. 6.4.
const SYSCLK: u32 = SYSTEM + 0x58;
const NOP: u32 = 0x0000_0013; // RISC-V Unprivileged ISA, addi x0,x0,0.
const EBREAK: u32 = 0x0010_0073;
const EFUSE_WORDS: usize = (0x17c - 0x2c) / size_of::<u32>(); // efuse_reg.h.
const FLASH: &[u8] = include_bytes!("data/esp_peripherals/flash.bin");

#[derive(Default)]
struct Probe(Vec<u32>);

impl Probe {
    fn pc(&self) -> u32 {
        ROM + self.0.len() as u32 * 4
    }

    fn li(&mut self, rd: u32, value: u32) {
        // RISC-V: LUI и ADDI со знаковыми младшими 12 битами.
        self.0.extend([
            (value.wrapping_add(0x800) & !0xfff) | rd << 7 | 0x37,
            (value & 0xfff) << 20 | rd << 15 | rd << 7 | 0x13,
        ]);
    }

    fn put(&mut self, addr: u32, value: u32) {
        self.li(5, addr);
        self.li(6, value);
        self.0.push(0x0062_a023); // sw t1,0(t0).
    }

    fn expect(&mut self, addr: u32, value: u32) {
        self.li(5, addr);
        self.0.push(0x0002_a383); // lw t2,0(t0).
        self.li(6, value);
        self.0.extend([0x0063_8463, EBREAK]); // beq t2,t1,+8.
    }

    fn wait_spi(&mut self) {
        self.li(5, SPI + 0x40); // spi_reg.h: SPI_DMA_INT_ST.
        self.0.extend([0x0002_a383, 0xfe03_8ee3]); // lw t2,0(t0); beq t2,x0,−4.
    }

    fn board(&self) -> Box<dyn BoardSim> {
        let mut rom: Vec<_> = self.0.iter().flat_map(|word| word.to_le_bytes()).collect();
        rom.resize(ROM_LEN, 0);
        let mut board = new_board(BoardId::Esp32C3);
        board
            .load_esp_flash(
                FLASH,
                esp_image::DEFAULT_PARTITION_TABLE_OFFSET,
                &rom,
                &[0; EFUSE_WORDS],
                [true; 3],
            )
            .unwrap();
        board
    }
}

fn gpio_out(pin: u32) -> u32 {
    GPIO + 0x554 + 4 * pin
}
fn gpio_in(signal: u32) -> u32 {
    GPIO + 0x154 + 4 * signal
}
fn pad(pin: u32) -> u32 {
    MUX + 4 + 4 * pin
}

#[test]
fn matrix_ие_инверсия_константы_и_разомкнутый_сток() {
    let mut p = Probe::default();
    // io_mux_reg.h: GPIO_FUNC=1, FUN_IE=1. gpio_reg.h: FUNC97 напрямую
    // соединяется с выходом 97 (TRM табл. 5.11-1).
    p.put(pad(2), 0x1200);
    p.put(pad(3), 0x1200);
    p.put(gpio_in(97), 0x40 | 2); // GPIO_SIG97_IN_SEL, GPIO2.
    p.put(gpio_out(3), 97 | 1 << 8); // GPIO_FUNC3_OUT_INV_SEL.
    // TRM табл. 5.12-1: подтяжки входов GPIO9, GPIO12…17 и U0RXD.
    const RESET_HIGH: u32 = (1 << 9) | (0x3f << 12) | (1 << 20);
    p.expect(GPIO + 0x3c, RESET_HIGH | (1 << 2)); // GPIO_IN: на 3 инверсия высокого GPIO2.
    p.put(gpio_in(97), 0x40 | 0x20 | 0x1e); // CONST_ONE с INPUT_INV → 0.
    p.expect(GPIO + 0x3c, RESET_HIGH | (1 << 2) | (1 << 3));
    p.put(GPIO + 0x80, 1 << 2); // GPIO_PIN3_PAD_DRIVER: высокий вывод отключается.
    p.expect(GPIO + 0x3c, RESET_HIGH | (1 << 2));
    p.put(gpio_out(3), 97 | 1 << 9 | 1 << 10); // OE=~GPIO_ENABLE[3].
    let drive_low_at = p.0.len() as u64;
    p.put(GPIO + 0x24, 1 << 3); // W1TS отключает инвертированный OE.
    let release_at = p.0.len() as u64;
    p.put(gpio_in(97), 0x40 | 22); // 22 вне диапазона 0…21.
    let mut board = p.board();
    board
        .apply(InputEvent {
            cycle: 0,
            input: BoardInput::Gpio { pin: 2, high: true },
        })
        .unwrap();
    board.run_cycles(drive_low_at).unwrap();
    assert_eq!(board.pin_output(3), Some(false));
    board.run_cycles(release_at - drive_low_at).unwrap();
    assert_eq!(board.pin_output(3), None);
    assert!(
        matches!(board.run_cycles(5), Err(BoardFault::Misconfigured(s))
        if s.contains("недоступный источник входа"))
    );
}

#[test]
fn ledc_apb_xtal_и_переключение_cpu() {
    // ledc_ll_set_slow_clk_sel: 1=APB, 3=XTAL; TRM §6.2.4.1: CPU/APB=20 МГц
    // при сбросе, при переходе на XTAL без делителя CPU/APB=40 МГц.
    for (source, hz) in [(1, 20_000_000), (3, 40_000_000)] {
        let mut p = Probe::default();
        p.put(CLK_EN, CLK_RESET | 1 << 11); // system_reg.h: LEDC_CLK_EN.
        p.put(pad(3), 0x1000);
        p.put(gpio_out(3), 45 | 1 << 8); // gpio_sig_map.h: LEDC0, инвертирован.
        p.put(LEDC + 0xd0, source);
        p.put(LEDC + 0xa0, 1 << 25 | (384 << 4) | 3); // Уставка: /1.5, 3 бита.
        p.put(LEDC + 8, 2 << 4); // DUTY=2/8, по выходу 6/8 из-за инверсии.
        p.put(LEDC + 12, 0xc010_0400); // START, INC, NUM=1, CYCLE=1, SCALE=0.
        // Штатный RMW записывает CONF0 по частям и затем PARA_UP.
        p.put(LEDC, 1 << 2);
        p.expect(LEDC, 1 << 2);
        p.put(LEDC, 1 << 4 | 1 << 2); // PARA_UP и SIG_OUT_EN (таймер 0).
        p.0.extend([NOP; 12]); // Пропускаем первый overflow, на котором защёлкивается канал.
        let first = p.0.len() as u64 - 1; // EBREAK внутри expect пропущен.
        p.0.extend([NOP; 12]); // 12 тактов источника за период, при 2*CPU два периода.
        p.put(SYSCLK, 0);
        let changed = p.0.len() as u64 - 1; // EBREAK внутри expect пропущен.
        p.0.push(NOP);
        let mut board = p.board();
        board.run_cycles(first).unwrap();
        let pwm = board.pwm(3).unwrap();
        assert_eq!(pwm.duty, 0.75);
        assert_eq!(pwm.period_s, 12.0 / f64::from(hz));
        let levels: Vec<_> = (0..12)
            .map(|_| {
                let level = board.pin_output(3).unwrap();
                board.run_cycles(1).unwrap();
                level
            })
            .collect();
        let counts = levels.iter().filter(|&&high| high).count();
        assert_eq!(counts, if source == 1 { 9 } else { 8 });
        board.run_cycles(changed + 1 - board.cycles()).unwrap();
        assert_eq!(board.pwm(3).unwrap().period_s, 12.0 / 40_000_000.0);
    }
}

#[test]
fn ledc_уставка_sdk_защёлкивается_на_переполнении() {
    let mut p = Probe::default();
    p.put(CLK_EN, CLK_RESET | 1 << 11);
    p.put(pad(3), 0x1000);
    p.put(gpio_out(3), 45);
    p.put(LEDC + 0xd0, 2); // Временный источник ledc_channel_config до настройки таймера.
    p.put(LEDC + 0xd0, 3); // XTAL. TRM §32.3.2 и ledc_ll_set_slow_clk_sel.
    // Т2.21 QEMU: divider=0x9c4 (2500/256), 10 бит, DUTY=0x1000 (256/1024).
    p.put(LEDC + 0xa0, 1 << 25 | 0x9c40 | 10);
    let timer_start = p.0.len() as u64 - 1;
    p.put(LEDC + 8, 256 << 4);
    p.put(LEDC, 1 << 4 | 1 << 2);
    // XTAL 40 МГц × 250 мкс = 10000 фронтов. При CPU 20 МГц это 5000 тактов.
    const PERIOD: u64 = 5000;
    p.0.resize((timer_start + PERIOD + 1) as usize, NOP);
    p.put(LEDC + 8, 768 << 4);
    p.put(LEDC, 1 << 4 | 1 << 2);
    let updated = p.0.len() as u64;
    p.0.resize((timer_start + 2 * PERIOD + 1) as usize, NOP);
    p.put(LEDC + 0xd0, 2); // Неизвестная частота RC_FAST при работающем таймере.
    let mut board = p.board();
    board.run_cycles(timer_start + PERIOD - 1).unwrap();
    assert!(board.pwm(3).is_none());
    board.run_cycles(1).unwrap();
    let pwm = board.pwm(3).unwrap();
    assert_eq!(pwm.period_s, 0.00025);
    assert_eq!(pwm.high_s, 0.0000625);
    assert_eq!(board.pin_output(3), Some(true));
    board.run_cycles(updated - board.cycles()).unwrap();
    assert_eq!(board.pwm(3), Some(pwm));
    board
        .run_cycles(timer_start + 2 * PERIOD - updated)
        .unwrap();
    assert_eq!(board.pwm(3).unwrap().duty, 0.75);
    assert!(
        matches!(board.run_cycles(6), Err(BoardFault::Unsupported(s)) if s.contains("источник тактов LEDC"))
    );
}

fn spi_probe(mode: SpiMode, valid_cs: bool, dma: bool) -> (Probe, u32) {
    let mut p = Probe::default();
    p.put(CLK_EN, CLK_RESET | 1 << 6 | if dma { 1 << 22 } else { 0 });
    // esp32c3/include/soc/spi_pins.h: SPI2_FUNC_NUM_QUAD=2.
    for (pin, mux) in [(2, 0x2200), (6, 0x2000), (7, 0x2000)] {
        p.put(pad(pin), mux);
    }
    if valid_cs {
        p.put(pad(10), 0x2000);
    }
    let (idle_high, out_edge) = match mode {
        SpiMode::Mode0 => (0, 0),
        SpiMode::Mode1 => (0, 1 << 9),
        SpiMode::Mode2 => (1 << 29, 1 << 9),
        SpiMode::Mode3 => (1 << 29, 0),
    };
    // spi_reg.h и spi_ll_master_set_mode. Источник XTAL, pre=1, N=4, high=2.
    p.put(SPI + 0xe8, 3);
    p.put(SPI + 0x0c, 3 << 12 | 1 << 6 | 3);
    p.put(SPI + 0x20, idle_high | 0x3e); // Активен только CS0.
    p.put(SPI + 0x10, 0x1800_0001 | out_edge); // MOSI, MISO, DOUTDIN.
    // ESP-IDF v6.1.1 в QEMU: spi_ll_set_addr_bitlen(0) и
    // spi_ll_set_command_bitlen(0) пишут все единицы в неактивные поля.
    p.put(SPI + 0x14, 0xf800_0000);
    p.put(SPI + 0x18, 0xf000_0000);
    p.put(SPI + 0x04, 0);
    p.put(SPI + 0x1c, 15);
    p.put(SPI + 0x98, 0x1253);
    p.put(SPI + 0x34, 1 << 12); // TRANS_DONE_ENA.
    p.put(SPI + 0x3c, 1 << 12); // spi_ll_set_int_stat.
    p.expect(SPI + 0x40, 1 << 12);
    p.put(SPI + 0x3c, 0); // spi_ll_clear_int_stat.
    p.expect(SPI + 0x40, 0);
    if dma {
        // system_reg.h DMA_CLK_EN/DMA_RST; gdma_reg.h DMA_INT_ENA_CH0.
        const GDMA_IRQ_ENA: u32 = 0x6003_f008;
        const SYS1_RESET: u32 = 0x1fe;
        p.expect(SYSTEM + 0x1c, SYS1_RESET);
        p.put(SYSTEM + 0x14, 1 << 6);
        p.put(SYSTEM + 0x1c, SYS1_RESET & !(1 << 6));
        p.put(GDMA_IRQ_ENA, 1 << 1); // DMA_IN_SUC_EOF_INT_ENA.
        p.expect(GDMA_IRQ_ENA, 1 << 1);
        p.put(SYSTEM + 0x1c, SYS1_RESET);
        p.expect(GDMA_IRQ_ENA, 0);
        p.put(SYSTEM + 0x1c, SYS1_RESET & !(1 << 6));
        p.put(SPI + 0x30, (1 << 28) | (1 << 27));
    }
    p.put(SPI, 1 << 23); // CMD.UPDATE.
    p.put(SPI, 1 << 24); // CMD.USR.
    let trigger = p.pc() - 4;
    p.wait_spi();
    p.expect(SPI + 0x98, 0x43e1);
    (p, trigger)
}

#[test]
fn spi_четыре_режима_с_прямым_mux_и_выходным_irq() {
    for mode in [
        SpiMode::Mode0,
        SpiMode::Mode1,
        SpiMode::Mode2,
        SpiMode::Mode3,
    ] {
        let (mut p, _) = spi_probe(mode, true, false);
        let end = p.pc();
        p.0.push(EBREAK);
        let mut board = p.board();
        assert!(board.run_cycles(1000).unwrap().bus_request_pending);
        let [event] = board.take_events().try_into().unwrap();
        let BoardOutput::Spi(req) = event.output else {
            panic!("{event:?}")
        };
        assert_eq!(req.mode, mode);
        assert_eq!(req.write, [0x53, 0x12]);
        // SCLK=40 МГц/4, CPU=20 МГц: 2 такта/бит, 16 периодов с интервалами CS.
        assert_eq!(req.cycles_per_bit, 2);
        assert_eq!(req.complete_cycle - event.cycle, 32);
        board
            .apply(InputEvent {
                cycle: board.cycles(),
                input: BoardInput::SpiReply {
                    bus: 2,
                    id: req.id,
                    status: TransferStatus::Complete,
                    bytes: vec![0xe1, 0x43],
                },
            })
            .unwrap();
        board
            .run_cycles(req.complete_cycle - board.cycles() - 1)
            .unwrap();
        assert!(board.take_events().is_empty());
        assert_eq!(
            board.run_cycles(100),
            Err(BoardFault::Breakpoint { pc: end })
        );
        let [done, stop] = board.take_events().try_into().unwrap();
        assert_eq!(
            stop.output,
            BoardOutput::Fault(BoardFault::Breakpoint { pc: end })
        );
        assert_eq!(done.cycle, req.complete_cycle);
        assert!(matches!(
            done.output,
            BoardOutput::SpiDone {
                status: TransferStatus::Complete,
                ..
            }
        ));
    }
}

#[test]
fn spi_требует_маршрут_cs_и_доступ_gdma_к_sram() {
    for (valid_cs, dma, diagnostic) in [
        (false, false, "выбранного CS"),
        (true, true, "SRAM не передана"),
    ] {
        let (p, trigger) = spi_probe(SpiMode::Mode0, valid_cs, dma);
        let mut board = p.board();
        let err = board.run_cycles(1000).unwrap_err();
        assert!(err.to_string().contains(diagnostic), "{err}");
        assert_eq!(board.esp_boot_position().unwrap().pc, trigger);
        let [stop] = board.take_events().try_into().unwrap();
        assert_eq!(stop.output, BoardOutput::Fault(err));
    }
}

#[test]
#[ignore = "нужны SPI flash ESP-IDF v6.1.1 и захват QEMU 9.2.2"]
fn штатная_rom_проходит_прямой_mux_uart0() {
    let read = |key| std::fs::read(std::env::var(key).expect(key)).expect(key);
    let fuse = read("SIM_ESP_BOOT_READBACK");
    assert_eq!(fuse.len(), EFUSE_WORDS * size_of::<u32>());
    let fuse: Vec<_> = fuse
        .chunks_exact(4)
        .map(|word| u32::from_le_bytes(word.try_into().unwrap()))
        .collect();
    let uart = String::from_utf8(read("SIM_ESP_BOOT_QEMU_UART")).unwrap();
    assert!(
        uart.split_once("Calling app_main()")
            .unwrap()
            .1
            .contains("Hello world!")
    );
    let mut board = new_board(BoardId::Esp32C3);
    board
        .load_esp_flash(
            &read("SIM_ESP_BOOT_FLASH"),
            esp_image::DEFAULT_PARTITION_TABLE_OFFSET,
            &read("SIM_ESP_BOOT_ROM"),
            &fuse,
            [false, false, true],
        )
        .unwrap();
    let trace = String::from_utf8(read("SIM_ESP_BOOT_QEMU_PC")).unwrap();
    let mut pcs = trace
        .lines()
        .filter(|line| line.starts_with("Trace "))
        .map(|line| {
            let field = line.split_once('[').unwrap().1.split('/').nth(1).unwrap();
            u32::from_str_radix(field, 16).unwrap()
        });
    // Наблюдение Т2.10: PLL в QEMU, XTAL/2 по TRM §6.2.4.1.
    const MATCHED: u64 = 5701;
    for step in 0..MATCHED {
        assert_eq!(
            board.esp_boot_position(),
            Some(EspBootPosition {
                step,
                pc: pcs.next().unwrap()
            })
        );
        assert_eq!(board.run_cycles(1).unwrap().cycle, step + 1);
    }
    assert_eq!(pcs.next(), Some(0x4004_988e));
    assert_eq!(
        board.esp_boot_position(),
        Some(EspBootPosition {
            step: MATCHED,
            pc: 0x4004_9872
        })
    );
    // Т2.10, io_mux_reg.h: запись PERIPHS_IO_MUX_U0RXD_U при PC 0x4004c29c.
    const PREVIOUS: u64 = 5771;
    board.run_cycles(PREVIOUS - MATCHED).unwrap();
    assert_eq!(
        board.esp_boot_position(),
        Some(EspBootPosition {
            step: PREVIOUS,
            pc: 0x4004_c29c
        })
    );
    assert_eq!(board.run_cycles(1).unwrap().cycle, PREVIOUS + 1);
    // usb_serial_jtag_reg.h: USB_SERIAL_JTAG_EP1_CONF_REG, свободный IN FIFO.
    const USB_READ: u64 = 6157;
    board.run_cycles(USB_READ - board.cycles()).unwrap();
    assert_eq!(
        board.esp_boot_position(),
        Some(EspBootPosition {
            step: USB_READ,
            pc: 0x4004_c90e
        })
    );
    assert_eq!(board.run_cycles(1).unwrap().cycle, USB_READ + 1);

    // QEMU ждёт USB, пустой EP1 в BoardSim принимает байт. Обе ветви
    // доходят до первого чтения UART_STATUS_REG (uart_reg.h).
    const UART_READ: u64 = 6195;
    const UART_PC: u32 = 0x4004_c0d4;
    assert_eq!(
        pcs.enumerate()
            .find(|&(_, pc)| pc == UART_PC)
            .map(|(index, _)| MATCHED + 2 + index as u64),
        Some(391_182)
    );
    board.run_cycles(UART_READ - board.cycles()).unwrap();
    assert_eq!(
        board.esp_boot_position(),
        Some(EspBootPosition {
            step: UART_READ,
            pc: UART_PC,
        })
    );
    let fault = BoardFault::UnsupportedRegister {
        pc: UART_PC,
        addr: 0x6000_001c,
        size: 4,
        write: false,
        block: "UART: не заданы уровни линий UART_STATUS",
    };
    assert_eq!(board.run_cycles(1), Err(fault.clone()));
    assert_eq!(board.run_cycles(1), Err(fault));
    assert_eq!(board.cycles(), UART_READ);
}
