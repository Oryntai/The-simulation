//! Сквозные проверки raw flash, начиная с инструкции в векторе сброса.

use sim_boards::{
    BoardFault, BoardId, BoardInput, BoardOutput, BoardSim, EspBootOptions, EspBootPosition,
    EspResetProfile, EspResetValueKind, InputEvent, ResetCause, SystimerPhase,
    esp_flash::EspFlashReadback, esp_image, new_board,
};
use std::io::{BufRead, BufReader};

// TRM §3.3.2 и ESP-IDF soc/esp32c3/register/soc/{reg_base,efuse_reg}.h.
const ROM: u32 = 0x4000_0000;
const ROM_LEN: usize = 384 * 1024;
const SRAM: u32 = 0x4038_0000;
const EFUSE_WORDS: usize = (0x17c - 0x2c) / size_of::<u32>();

// ESP-IDF soc/esp32c3/register/soc/{system,extmem,spi_mem}_reg.h,
// soc/esp32c3/include/soc/ext_mem_defs.h.
const SYSCLK: u32 = 0x600c_0058;
const CACHE_ENABLE: u32 = 0x600c_4000;
const CACHE_BUS: u32 = 0x600c_4004;
const MMU_FIRST: u32 = 0x600c_5000;
const SPI0_CACHE: u32 = 0x6000_303c;
const SRAM_USAGE: u32 = 0x600c_101c;
const DROM: u32 = 0x3c00_0000;
const IROM: u32 = 0x4200_0000;
const SRAM0: u32 = 0x4037_c000;
const SPI1: u32 = 0x6000_2000;

const EBREAK: u32 = 0x0010_0073; // RISC-V Unprivileged ISA, EBREAK.
const WFI: u32 = 0x1050_0073; // RISC-V Privileged ISA, WFI.
const MRET: u32 = 0x3020_0073; // RISC-V Privileged ISA, MRET.
const JUMP_BACK: u32 = 0xffdf_f06f; // RISC-V Unprivileged ISA, jal x0,-4.

#[derive(Default)]
struct Probe(Vec<u32>);

impl Probe {
    fn pc(&self) -> u32 {
        ROM + size_of_val(self.0.as_slice()) as u32
    }

    fn li(&mut self, rd: u32, value: u32) {
        // RISC-V ISA, LUI + ADDI. Младшие 12 бит знаковые.
        let high = value.wrapping_add(0x800) & !0xfff;
        let low = value & 0xfff;
        self.0
            .extend([high | rd << 7 | 0x37, low << 20 | rd << 15 | rd << 7 | 0x13]);
    }

    fn put(&mut self, addr: u32, value: u32) {
        self.li(5, addr);
        self.li(6, value);
        self.0.push(0x0062_a023); // sw t1,0(t0)
    }

    fn read(&mut self, addr: u32) -> u32 {
        self.li(5, addr);
        let pc = self.pc();
        self.0.push(0x0002_a383); // lw t2,0(t0)
        pc
    }

    fn jump(&mut self, addr: u32, link: bool) {
        self.li(5, addr);
        self.0.push(0x0002_8067 | (u32::from(link) << 7)); // jalr ra/x0,0(t0).
    }

    fn expect_read(&mut self, addr: u32, expected: u32) {
        self.read(addr);
        self.li(6, expected);
        // beq t2,t1,+8. При неверном чтении будет другой PC остановки.
        self.0.extend([0x0063_8463, EBREAK]);
    }

    fn image(&self) -> Vec<u8> {
        let mut rom: Vec<_> = self.0.iter().flat_map(|word| word.to_le_bytes()).collect();
        rom.resize(ROM_LEN, 0);
        rom
    }
}

fn raw_image(words: &[u32]) -> Vec<u8> {
    // esp_app_format.h: 1 сегмент, DIO, 2 МиБ и 80 МГц, CHIP_ID ESP32-C3.
    let mut bin = vec![0xe9, 1, 2, 0x1f];
    bin.extend(SRAM.to_le_bytes());
    bin.extend([0xee, 0, 0, 0, 5, 0, 0, 0, 0, 199, 0, 0, 0, 0, 0, 0]);
    bin.extend(SRAM.to_le_bytes());
    bin.extend((size_of_val(words) as u32).to_le_bytes());
    let payload: Vec<_> = words.iter().flat_map(|word| word.to_le_bytes()).collect();
    bin.extend(&payload);
    // esp_image_format.c: XOR payload и выравнивание контрольной суммы.
    bin.resize(bin.len() / 16 * 16 + 15, 0);
    bin.push(payload.into_iter().fold(0xef, |a, b| a ^ b));
    bin
}

fn board_for(probe: &Probe, phase: Option<SystimerPhase>) -> Box<dyn BoardSim> {
    board_with_loader(probe, phase, &[EBREAK], &[EBREAK])
}

fn board_with_loader(
    probe: &Probe,
    phase: Option<SystimerPhase>,
    loader: &[u32],
    app: &[u32],
) -> Box<dyn BoardSim> {
    probe_board(probe, phase, loader, app, None)
}

fn probe_board(
    probe: &Probe,
    phase: Option<SystimerPhase>,
    loader: &[u32],
    app: &[u32],
    profile: Option<EspResetProfile>,
) -> Box<dyn BoardSim> {
    const TABLE: u32 = esp_image::DEFAULT_PARTITION_TABLE_OFFSET;
    // partitions_singleapp.csv и gen_esp32part.py: заводской app с 0x10000.
    const APP: u32 = 0x10000;
    let mut flash = raw_image(loader);
    flash.resize(TABLE as usize, 0xff);
    flash.extend([0xaa, 0x50, 0, 0]);
    flash.extend(APP.to_le_bytes());
    flash.extend(APP.to_le_bytes());
    flash.extend(b"factory");
    flash.resize(TABLE as usize + 32, 0);
    flash.resize(APP as usize, 0xff);
    flash.extend(raw_image(app));
    let mut board = new_board(BoardId::Esp32C3);
    let options = EspBootOptions {
        systimer_phase: phase,
        flash_readback: None,
    };
    let rom = probe.image();
    let fuse = [0; EFUSE_WORDS];
    let straps = [true; 3];
    match profile {
        Some(profile) => board.load_esp_flash_with_profile(
            &flash,
            TABLE,
            &rom,
            &fuse,
            straps,
            options.with_reset_profile(profile),
        ),
        None => board.load_esp_flash_with_options(&flash, TABLE, &rom, &fuse, straps, options),
    }
    .unwrap();
    board
}

#[test]
fn rom_читает_spi_flash_и_настраивает_extmem() {
    for enable_cache in [false, true] {
        let mut probe = Probe::default();
        probe.expect_read(CACHE_BUS, 3); // EXTMEM_ICACHE_SHUT_{I,D}BUS, сброс 1.
        probe.expect_read(MMU_FIRST, 0x100); // INVALID, ext_mem_defs.h.
        probe.put(MMU_FIRST, 0); // Отображение на физическую страницу ноль.
        probe.expect_read(MMU_FIRST, 0);
        // spi_mem_reg.h: ADDR=0, MISO_DLEN=31, FLASH_READ=1, ответ в W0.
        probe.put(SPI1 + 4, 0);
        probe.put(SPI1 + 0x28, u32::BITS - 1);
        probe.put(SPI1, 1 << 31);
        let header =
            u32::from_le_bytes(raw_image(&[EBREAK])[..size_of::<u32>()].try_into().unwrap());
        probe.expect_read(SPI1 + 0x58, header);
        // SRAM0 CPU до передачи банка кэшу, TRM §3.3.2 и §14.4.2.1.
        probe.put(SRAM0, EBREAK);
        probe.expect_read(SRAM0, EBREAK);
        if enable_cache {
            // TRM §14.4.2.1: CPU_CACHE=0 передаёт SRAM0 кэшу.
            probe.put(SRAM_USAGE, 0);
            probe.put(SPI0_CACHE, 1); // SPI_MEM_CACHE_REQ_EN.
            probe.put(CACHE_BUS, 0); // Снятие масок обеих шин.
            probe.put(CACHE_ENABLE, 1); // EXTMEM_ICACHE_ENABLE.
        }
        let read_pc = probe.read(DROM);
        probe.expect_read(DROM, header);
        let success_pc = probe.pc();
        probe.0.push(EBREAK);
        let mut board = board_for(&probe, None);
        let stop = if enable_cache {
            BoardFault::Breakpoint { pc: success_pc }
        } else {
            BoardFault::BusError {
                pc: read_pc,
                addr: DROM,
                write: false,
            }
        };
        assert_eq!(board.run_cycles(probe.0.len() as u64), Err(stop.clone()));
        assert_eq!(
            board.esp_boot_position().unwrap().pc,
            if enable_cache { success_pc } else { read_pc }
        );
        assert_eq!(board.run_cycles(1), Err(stop));
    }
}

#[test]
fn загрузчик_исполняет_irom_и_rom_с_реальным_abi() {
    // ESP-IDF esp32c3.rom.libc-suboptimal_for_misaligned_mem.ld, memcpy.
    const MEMCPY: u32 = 0x4000_0358;
    // TRM табл. 3.3-1: алиас DBUS SRAM1; смещение двух ячеек стенда.
    const DRAM: u32 = 0x3fc8_0000;
    const SRC: u32 = DRAM + 0x400;
    const DST: u32 = SRC + size_of::<u32>() as u32;
    const CALLBACK: u32 = SRAM + 0x200; // Отдельная ячейка кода стенда в SRAM1.
    // esp_app_format.h: 24 байта заголовка образа и 8 заголовка сегмента.
    const APP_CODE: u32 = IROM + 24 + 8;
    // Синтетические эффекты стенда: IROM возвращает 37, memcpy читает байт 0x5a.
    const APP_WORDS: [u32; 2] = [0x0250_0513, 0x0000_8067]; // li a0,37; ret.
    const ROM_WORDS: [u32; 6] = [
        0x0000_8393, // mv t2,ra — сохранить адрес возврата загрузчика.
        0x0004_00e7, // jalr ra,0(s0) — обратный вызов гостя.
        0x0003_8093, // mv ra,t2.
        0x0005_c283, // lbu t0,0(a1).
        0x0055_0023, // sb t0,0(a0).
        0x0000_8067, // ret.
    ];
    for valid in [false, true] {
        let mut loader = Probe::default();
        loader.put(SRAM_USAGE, 0);
        loader.put(SPI0_CACHE, 1);
        loader.put(CACHE_BUS, 0);
        loader.put(CACHE_ENABLE, 1);
        if valid {
            loader.put(MMU_FIRST, 1); // Смещение app из board_with_loader / 64 КиБ.
        }
        loader.jump(APP_CODE, true);
        loader.li(6, 37);
        loader.0.extend([0x0065_0463, EBREAK]); // beq a0,t1,+8.
        loader.li(10, DST); // RV32 psABI: a0, a1, a2 передают первые три аргумента.
        loader.li(11, SRC);
        loader.li(12, 1);
        loader.li(8, CALLBACK);
        loader.jump(MEMCPY, true);
        loader.li(6, DST);
        loader.0.extend([0x0065_0463, EBREAK]); // Проверка возврата a0.
        loader.expect_read(DST, 0x5a);
        let success = SRAM + size_of_val(loader.0.as_slice()) as u32;
        loader.0.push(EBREAK);

        // ROM стенда размещает копирование загрузчика после заглушки функции.
        const ROM_COPY: u32 = ROM + 0x1000;
        let mut rom = Probe::default();
        rom.jump(ROM_COPY, false);
        rom.0
            .resize(((MEMCPY - ROM) / size_of::<u32>() as u32) as usize, 0);
        rom.0.extend(ROM_WORDS);
        rom.0
            .resize(((ROM_COPY - ROM) / size_of::<u32>() as u32) as usize, 0);
        rom.put(SRC, 0x5a);
        rom.put(DST, 0);
        rom.put(DRAM + (CALLBACK - SRAM), 0x0000_8067); // ret в тело ROM.
        for (index, &instruction) in loader.0.iter().enumerate() {
            rom.put(DRAM + index as u32 * size_of::<u32>() as u32, instruction);
        }
        rom.jump(SRAM, false);
        let mut board = board_with_loader(&rom, None, &loader.0, &APP_WORDS);
        let fault = if valid {
            BoardFault::Breakpoint { pc: success }
        } else {
            BoardFault::BusError {
                pc: APP_CODE,
                addr: APP_CODE,
                write: false,
            }
        };
        assert_eq!(
            board.run_cycles((rom.0.len() + loader.0.len()) as u64),
            Err(fault)
        );
    }
}

#[test]
fn usb_ep1_без_хоста_держит_занятый_буфер_до_сброса() {
    // TRM §30.3.2 и usb_serial_jtag_reg.h: пакет 64 байта либо WR_DONE.
    const FIFO: u32 = 0x6004_3000;
    const CONF: u32 = FIFO + 4;
    const IN_FREE: u32 = 1 << 1;
    const WR_DONE: u32 = 1;
    const PACKET: usize = 64;
    for flush in [false, true] {
        let mut p = Probe::default();
        p.expect_read(CONF, IN_FREE);
        for _ in 0..if flush { 1 } else { PACKET - 1 } {
            p.put(FIFO, b'A'.into()); // Байт стенда.
        }
        p.expect_read(CONF, IN_FREE);
        if flush {
            p.put(CONF, WR_DONE);
        } else {
            p.put(FIFO, b'B'.into());
        }
        p.expect_read(CONF, 0);
        p.put(FIFO, b'C'.into()); // Буфер занят, пока его не прочитает хост.
        let fault = BoardFault::UnsupportedRegister {
            pc: p.pc() - 4,
            addr: FIFO,
            write: true,
            size: 4,
            block: "USB Serial/JTAG EP1 без хоста",
        };
        let mut board = board_for(&p, None);
        for _ in 0..2 {
            assert_eq!(board.run_cycles(p.0.len() as u64), Err(fault.clone()));
            board.reset();
        }
    }
}

#[test]
fn системный_затвор_ram_uart_блокирует_доступ_к_fifo() {
    // TRM рег. 16.3 и 26.1, ESP-IDF system_reg.h и uart_reg.h:
    // убираем из образа сброса SYSREG только SYSTEM_UART_MEM_CLK_EN[24].
    const SYS_CLOCK_ENABLE: u32 = 0x600c_0010;
    const SYS_CLOCK_RESET: u32 = 0xf9c1_e06f;
    const UART_MEM_CLOCK: u32 = 1 << 24;
    const UART0_FIFO: u32 = 0x6000_0000;
    let mut probe = Probe::default();
    probe.expect_read(SYS_CLOCK_ENABLE, SYS_CLOCK_RESET);
    probe.put(SYS_CLOCK_ENABLE, SYS_CLOCK_RESET & !UART_MEM_CLOCK);
    probe.expect_read(SYS_CLOCK_ENABLE, SYS_CLOCK_RESET & !UART_MEM_CLOCK);
    probe.put(UART0_FIFO, u32::from(b'A'));
    let mut board = board_for(&probe, None);
    assert_eq!(
        board.run_cycles(probe.0.len() as u64),
        Err(BoardFault::Misconfigured(
            "UART: нет тактов SYSREG или активен сброс".into()
        ))
    );
    assert_eq!(board.esp_boot_position().unwrap().pc, probe.pc() - 4);
    assert!(board.uart_take().is_empty());
}

#[test]
fn время_и_снимок_переживают_смену_частоты() {
    let mut probe = Probe::default();
    probe.put(SYSCLK, 0); // SYSTEM_SOC_CLK_SEL=XTAL, SYSTEM_PRE_DIV_CNT=0.
    let switch = probe.0.len() as u64;
    let success_pc = probe.pc() + size_of::<u32>() as u32;
    probe.0.extend([0x0000_0013, EBREAK]); // nop; ebreak
    let mut board = board_for(&probe, None);

    // TRM §6.2.4.1: первые пять тактов по 50 нс, следующий такт 25 нс.
    let boundary = u128::from(switch) * 200; // Четверти нс.
    assert_eq!(
        board
            .run_until_quarter_ns(boundary - 1)
            .unwrap()
            .unwrap()
            .cycle,
        switch - 1
    );
    assert_eq!(
        board.run_until_quarter_ns(boundary).unwrap().unwrap().cycle,
        switch
    );
    let mut snapshot = board.clone_state();
    for item in [&mut board, &mut snapshot] {
        assert_eq!(
            item.run_until_quarter_ns(boundary + 99)
                .unwrap()
                .unwrap()
                .cycle,
            switch
        );
        assert_eq!(
            item.run_until_quarter_ns(boundary + 100)
                .unwrap()
                .unwrap()
                .cycle,
            switch + 1
        );
        assert_eq!(
            item.cycle_time_quarter_ns(switch - 1),
            Ok(Some(boundary - 200))
        );
        assert_eq!(
            item.cycle_time_quarter_ns(switch + 1),
            Ok(Some(boundary + 100))
        );
        assert!(item.cycle_time_quarter_ns(switch + 2).is_err());
        assert_eq!(
            item.run_cycles(1),
            Err(BoardFault::Breakpoint { pc: success_pc })
        );
        item.reset();
        assert_eq!(
            item.esp_boot_position(),
            Some(EspBootPosition { step: 0, pc: ROM })
        );
        assert_eq!(
            item.run_until_quarter_ns(boundary + 299)
                .unwrap()
                .unwrap()
                .cycle,
            switch + 1
        );
        assert_eq!(
            item.run_until_quarter_ns(boundary + 300)
                .unwrap()
                .unwrap()
                .cycle,
            switch + 2
        );
    }
    for bad in [f64::NAN, f64::INFINITY] {
        let mut board = board_for(&probe, None);
        assert!(matches!(
            board.run_for(bad),
            Err(BoardFault::Misconfigured(_))
        ));
        assert_eq!(board.cycles(), 0);
    }
}

#[test]
fn systimer_доходит_до_вектора_cpu_через_irq() {
    // ESP-IDF soc/esp32c3/register/soc/{systimer,interrupt_core0}_reg.h
    // и soc/esp32c3/include/soc/interrupts.h (TARGET0 — источник 37).
    const TIMER: u32 = 0x6002_3000;
    const MATRIX: u32 = 0x600c_2000;
    const LINE: u32 = 3; // Программируемая линия стенда.
    const VECTOR: u32 = ROM + 0x400; // TRM рег. mtvec, BASE выровнен на 256 байт.
    const HANDLER: u32 = VECTOR + LINE * size_of::<u32>() as u32;
    // Адрес вне реализованных CSR. В отличие от EBREAK, отказ модели не виден гостю.
    const STOP_CSR: u32 = 0xfff;
    const STOP: u32 = 0xfff0_22f3; // csrr t0,0xfff
    for enable in [0, 1 << LINE] {
        let mut probe = Probe::default();
        probe.li(5, VECTOR | 1); // mtvec.MODE = vectored по TRM §1.4.2.
        probe.0.push(0x3052_9073); // RISC-V: csrw mtvec,t0.
        probe.put(MATRIX + 37 * 4, LINE);
        probe.put(MATRIX + 0x114 + LINE * 4, 1);
        probe.put(MATRIX + 0x104, enable);
        probe.put(TIMER + 0x34, (1 << 30) | 1); // TARGET0: период 1 тик UNIT0.
        probe.put(TIMER + 0x50, 1); // COMP0_LOAD.
        probe.put(TIMER, (1 << 30) | (1 << 24)); // UNIT0_WORK_EN/TARGET0_WORK_EN.
        probe.put(TIMER + 0x64, 1); // TARGET0_INT_ENA.
        probe.expect_read(TIMER + 0x70, 1); // TARGET0_INT_ST поднят и без маршрута CPU.
        *probe.0.last_mut().unwrap() = STOP; // Ошибка сравнения не должна входить в trap.
        probe.0.push(0x3004_6073); // RISC-V: csrsi mstatus,8 (MIE).
        let main_pc = probe.pc();
        probe.0.push(STOP);
        let budget = probe.0.len() as u64 + 1; // Дополнительный шаг при входе в trap.
        probe
            .0
            .resize(((HANDLER - ROM) as usize) / size_of::<u32>(), 0);
        probe.0.push(0x3420_23f3); // csrr t2,mcause (RISC-V Privileged ISA).
        probe.li(6, (1 << 31) | LINE); // mcause.Interrupt и номер линии.
        probe.0.extend([0x0063_8463, STOP]); // beq t2,t1,+8
        let handler_pc = probe.pc();
        probe.0.push(STOP);
        let mut board = board_for(&probe, Some(SystimerPhase::FirstAfterTwoXtalCycles));
        let pc = if enable == 0 { main_pc } else { handler_pc };
        let handler_steps = u64::from((handler_pc - HANDLER) / size_of::<u32>() as u32);
        assert_eq!(
            board.run_cycles(budget + handler_steps),
            Err(BoardFault::UnsupportedRegister {
                pc,
                addr: STOP_CSR,
                write: false,
                size: size_of::<u32>() as u8,
                block: "CSR RV32",
            })
        );
    }
}

#[test]
fn systimer_требует_фазу_из_входов() {
    // ESP-IDF soc/esp32c3/register/soc/systimer_reg.h, SYSTIMER_CONF_REG.
    const CONF: u32 = 0x6002_3000;
    let mut probe = Probe::default();
    let pc = probe.read(CONF);
    let mut board = board_for(&probe, None);
    assert_eq!(
        board.run_cycles(probe.0.len() as u64),
        Err(BoardFault::UnsupportedRegister {
            pc,
            addr: CONF,
            write: false,
            size: size_of::<u32>() as u8,
            block: "SYSTIMER: не задана начальная фаза",
        }),
    );
}

#[test]
fn freertos_тик_будит_wfi_каждые_10_мс() {
    // ESP-IDF freertos/port_systick.c и Kconfig: UNIT1 → TARGET0, 100 Гц,
    // прерывание уровневое, в обработчике очищается TARGET0_INT.
    // TRM §10.3 и systimer_reg.h: 16 МГц дают 160000 тактов за 10 мс.
    const TIMER: u32 = 0x6002_3000;
    const MATRIX: u32 = 0x600c_2000;
    const LINE: u32 = 3; // Линия стенда.
    const VECTOR: u32 = ROM + 0x400; // TRM §1.4.2, выравнивание mtvec.
    const HANDLER: u32 = VECTOR + LINE * 4;
    const PERIOD_TICKS: u32 = 16_000_000 / 100;
    const PERIOD_CYCLES: u64 = 20_000_000 / 100; // TRM §6.2.4.1, XTAL/2.
    const CPU_PER_CONF: u32 = 0x600c_0008; // ESP-IDF system_reg.h.
    for (gate, expect_irq) in [(false, true), (true, false)] {
        let mut p = Probe::default();
        if gate {
            // PLL_FREQ_SEL сброс 1, WAIT_MODE_FORCE_ON=0, задержка=2 такта.
            p.put(CPU_PER_CONF, (1 << 2) | (2 << 4));
        }
        p.li(5, VECTOR | 1);
        p.0.push(0x3052_9073); // csrw mtvec,t0.
        p.put(MATRIX + 37 * 4, LINE); // ETS_SYSTIMER_TARGET0_INTR_SOURCE.
        p.put(MATRIX + 0x114 + LINE * 4, 1); // CPU_INT_PRI_n.
        p.put(MATRIX + 0x104, 1 << LINE); // CPU_INT_ENABLE.
        p.put(TIMER + 0x34, (1 << 31) | (1 << 30) | PERIOD_TICKS);
        p.put(TIMER + 0x50, 1); // COMP0_LOAD.
        p.put(TIMER + 0x64, 1); // TARGET0_INT_ENA.
        p.0.push(0x3004_6073); // csrsi mstatus,8 — MIE.
        p.put(TIMER, (1 << 29) | (1 << 26) | (1 << 24)); // UNIT1, STALL_EN, TARGET0.
        let first_enabled = p.0.len() as u64 - 1;
        p.0.push(WFI);
        let idle_pc = p.pc();
        p.0.push(JUMP_BACK);
        p.0.resize(((HANDLER - ROM) / 4) as usize, 0);
        p.put(TIMER + 0x6c, 1); // TARGET0_INT_CLR.
        p.0.push(MRET);
        let mut board = board_for(&p, Some(SystimerPhase::FirstAfterTwoXtalCycles));

        // Период содержит чётное число фронтов. Через ровно 200000 тактов
        // CPU после начала счёта уровень поднят, вход в trap занимает ещё шаг.
        for n in 1..=2 {
            let deadline = first_enabled + n * PERIOD_CYCLES;
            board.run_cycles(deadline - board.cycles()).unwrap();
            assert_eq!(board.esp_boot_position().unwrap().pc, idle_pc);
            board.run_cycles(1).unwrap();
            assert_eq!(
                board.esp_boot_position().unwrap().pc,
                if expect_irq { HANDLER } else { idle_pc }
            );
            assert_eq!(
                board.cycle_time_quarter_ns(deadline),
                Ok(Some(u128::from(deadline) * 200))
            );
        }
    }
}

#[test]
fn rwdt_поднимает_rtc_core_и_обработчик_снимает_уровень() {
    // ESP-IDF esp32c3/register/soc/{rtc_cntl,interrupt_core0}_reg.h,
    // interrupts.h (RTC_CORE = 27); TRM §12.2.2.2, hold STG0 умножается на 2 при eFuse=0.
    const RTC: u32 = 0x6000_8000;
    const MATRIX: u32 = 0x600c_2000;
    const LINE: u32 = 3; // Линия стенда.
    const VECTOR: u32 = ROM + 0x400; // TRM §1.4.2, выравнивание mtvec.
    const HANDLER: u32 = VECTOR + LINE * 4;
    const WDT_INT: u32 = 1 << 3;
    const HOLD: u32 = 2; // Уставка стенда в RTC_SLOW_CLK.
    // Строгая верхняя граница 1 периода дискретизации CPU поверх 4 тактов RC_SLOW.
    const WAIT: u64 = (20_000_000u64 * (2 * HOLD) as u64).div_ceil(136_000);
    let mut p = Probe::default();
    p.li(5, VECTOR | 1);
    p.0.push(0x3052_9073); // csrw mtvec,t0.
    p.put(MATRIX + 27 * 4, LINE);
    p.put(MATRIX + 0x114 + LINE * 4, 1);
    p.put(MATRIX + 0x104, 1 << LINE);
    p.put(RTC + 0xa8, 0x50d8_3aa1); // WDTWPROTECT, ключ TRM §12.2.2.3.
    p.put(RTC + 0x90, 0); // Остановить RWDT и отключить FLASHBOOT_MOD_EN.
    p.put(RTC + 0x94, HOLD); // WDT_STG0_HOLD.
    p.put(RTC + 0x40, WDT_INT); // RTC_CNTL_INT_ENA.
    p.put(RTC + 0x90, (1 << 31) | (1 << 28)); // WDT_EN и STG0=IRQ.
    p.0.push(0x3004_6073); // csrsi mstatus,8.
    p.0.push(WFI);
    let resume = p.pc();
    p.0.push(0xfff0_22f3); // csrr t0,0xfff — проверка возврата без ловушки EBREAK.
    let startup = p.0.len() as u64;
    p.0.resize(((HANDLER - ROM) / 4) as usize, 0);
    p.expect_read(RTC + 0x48, WDT_INT);
    p.expect_read(MATRIX + 0xf8, 1 << 27); // INTERRUPT_CORE0_INTR_STATUS_0.
    p.put(RTC + 0x4c, WDT_INT); // RTC_CNTL_INT_CLR.
    p.expect_read(RTC + 0x48, 0);
    p.expect_read(MATRIX + 0xf8, 0);
    p.0.push(MRET);
    let handler_budget = p.0.len() as u64 - u64::from((HANDLER - ROM) / 4);
    let mut board = board_for(&p, None);
    let stop = BoardFault::UnsupportedRegister {
        pc: resume,
        addr: 0xfff,
        write: false,
        size: 4,
        block: "CSR RV32",
    };
    assert_eq!(board.run_cycles(startup + WAIT + handler_budget), Err(stop));
}

/// Трасса QEMU 9.2.2 из `-accel tcg,one-insn-per-tb=on -d in_asm,exec,nochain`.
/// Остальные параметры, образ и UART описаны в firmware-tests/esp_sdk/SOURCES.md.
/// SIM_ESP_BOOT_READBACK — результат `pmemsave 0x6000882c 336 readback.bin` при -S.
/// SIM_ESP_BOOT_QEMU_MMIO — `-trace enable=memory_region_ops_write,file=mmio.log`.
/// При совмещённом логе событий MMIO и `-d exec` оба пути указывают на этот лог.
/// SIM_ESP_BOOT_PC_APP_MAIN — адрес `app_main` из `riscv32-esp-elf-nm hello_world.elf` в hex.
#[test]
#[ignore = "нужны штатный образ SDK, дамп ROM, eFuse, UART и трасса QEMU"]
fn штатный_образ_сверяется_с_qemu_на_общем_пути() {
    let uart = String::from_utf8(read_fixture("SIM_ESP_BOOT_QEMU_UART")).unwrap();
    assert!(
        uart.split_once("Calling app_main()")
            .unwrap()
            .1
            .contains("Hello world!")
    );

    let mut board = qemu_rom_board();
    // Из потактовой трассы. Первый разный PC следует за bnez в 0x40049870.
    // QEMU x/1wx 0x600c0058 даёт 0xa8401 (PLL), TRM §6.2.4.1 задаёт XTAL/2.
    const MATCHED: u64 = 5701;
    let mut pcs = qemu_pcs().enumerate();
    for step in 0..MATCHED {
        let (_, pc) = pcs.next().expect("полный общий префикс");
        assert_eq!(
            board.esp_boot_position(),
            Some(EspBootPosition { step, pc })
        );
        assert_eq!(board.run_cycles(1).unwrap().cycle, step + 1);
    }
    assert_eq!(pcs.next(), Some((MATCHED as usize, 0x4004_988e)));
    assert_eq!(
        board.esp_boot_position(),
        Some(EspBootPosition {
            step: MATCHED,
            pc: 0x4004_9872
        })
    );

    // ROM Espressif rev. 3: обе ветви ets_get_apb_freq возвращаются сюда.
    const RETURN: u32 = 0x4004_7c88;
    let (qemu_index, _) = pcs.find(|&(_, pc)| pc == RETURN).unwrap();
    // Длина ветки XTAL/2 по esp32c3_rev3_rom.elf, ets_get_apb_freq
    // вместе с вызовами ets_get_xtal_freq и ets_get_xtal_div.
    while board.cycles() < 5726 {
        board.run_cycles(1).unwrap();
    }
    assert_eq!((board.cycles() + 1, qemu_index + 1), (5727, 5707));

    // Первое чтение EP1_CONF с USB-печатью ROM. TRM §30.3.2: буфер пуст,
    // поэтому модель принимает первый байт. QEMU esp32c3_jtag.c возвращает 0 для EP1.
    const USB_BRANCH: u32 = 0x4004_c912;
    let mut qemu_step = qemu_index;
    let mut pc = RETURN;
    let mut joined = 0;
    loop {
        assert_eq!(
            board.esp_boot_position().unwrap().pc,
            pc,
            "PC QEMU #{}",
            qemu_step + 1
        );
        joined += 1;
        board.run_cycles(1).unwrap();
        // ROM: andi в 0x4004c912, затем bnez в 0x4004c914.
        if pc == USB_BRANCH + 2 {
            break;
        }
        (qemu_step, pc) = pcs.next().unwrap();
    }
    assert_eq!(joined, 434);
    assert_eq!(pcs.next(), Some((6140, 0x4004_c916)));
    assert_eq!(
        board.esp_boot_position(),
        Some(EspBootPosition {
            step: 6160,
            pc: 0x4004_c942
        })
    );

    // ROM rev. 3, usb_uart_tx_one_char: после записи пакета или исчерпания
    // ожидания обе ветви восстанавливают регистры в одной точке.
    const USB_RETURN: u32 = 0x4004_c948;
    let (qemu_usb_return, _) = pcs.find(|&(_, pc)| pc == USB_RETURN).unwrap();
    assert_eq!(qemu_usb_return + 1, 391149);
    board.run_cycles(2).unwrap(); // beqz; sw.
    let mut pc = USB_RETURN;
    let mut step = qemu_usb_return;
    let mut after_usb = 0;
    // esp32c3_rev3_rom.elf, uart_tx_one_char_uart.part.0: sw a1,0(a5).
    const UART_FIFO_WRITE: u32 = 0x4004_c0de;
    loop {
        assert_eq!(
            board.esp_boot_position().unwrap().pc,
            pc,
            "PC QEMU #{}",
            step + 1
        );
        after_usb += 1;
        if pc == UART_FIFO_WRITE {
            break;
        }
        board.run_cycles(1).unwrap();
        (step, pc) = pcs.next().unwrap();
    }
    assert_eq!(after_usb, 38);
    assert_eq!(step + 1, 391186);

    // Трасса QEMU -trace memory_region_ops_write: последняя настройка CONF0
    // перед первым байтом 0x45. ROM сбросила MEM_CLK_EN [28].
    let mmio = std::fs::File::open(std::env::var("SIM_ESP_BOOT_QEMU_MMIO").unwrap()).unwrap();
    let mut conf = None;
    let mut fifo = None;
    for line in BufReader::new(mmio).lines().map(Result::unwrap) {
        if let Some((_, value)) = line.split_once("addr 0x60000020 value ") {
            conf = value.split_whitespace().next().map(str::to_owned);
        }
        if let Some((_, value)) = line.split_once("addr 0x60000000 value ") {
            fifo = value.split_whitespace().next().map(str::to_owned);
            break;
        }
    }
    assert_eq!(conf.as_deref(), Some("0x400001c"));
    assert_eq!(fifo.as_deref(), Some("0x45"));

    let mmio = std::fs::File::open(std::env::var("SIM_ESP_BOOT_QEMU_MMIO").unwrap()).unwrap();
    let writes: Vec<_> = BufReader::new(mmio)
        .lines()
        .map(Result::unwrap)
        .filter(|line| line.starts_with("memory_region_ops_write "))
        .filter_map(|line| {
            let (_, fields) = line.split_once("addr 0x6000e")?;
            Some(fields.split_once(" size ").unwrap().0.to_owned())
        })
        .take(8)
        .collect();
    // rtc_clk_init и первый ROM RMW SAR; константа QEMU из esp32c3_io_read.
    // Значения QEMU подтверждают команды и порядок, а сброс кремния остаётся открытым.
    assert_eq!(
        writes,
        [
            "044 value 0xffffff",
            "044 value 0xfdffff",
            "044 value 0xfbffff",
            "048 value 0xffffff",
            "048 value 0xffff4f",
            "044 value 0xfffbffff",
            "000 value 0x4000769",
            "048 value 0xffff4f",
        ]
    );

    assert_eq!(board.cycles(), 6199);
    assert_eq!(board.run_cycles(1).unwrap().cycle, 6200);
    // Из прогона ROM rev. 3: assist_debug_clock_enable начинается перед lw a4,4(a5).
    const ASSIST_READ_PC: u32 = 0x4004_7c1e;
    const BOARD_STOP: u64 = 101474;
    const QEMU_STOP: usize = 400778;
    let mut board_tail = Vec::new();
    while board.cycles() < BOARD_STOP {
        let position = board.esp_boot_position().unwrap();
        board_tail.push(position.pc);
        board.run_cycles(1).unwrap_or_else(|fault| {
            panic!(
                "шаг {}, PC {:#010x}: {fault}",
                position.step + 1,
                position.pc
            )
        });
    }
    let stop = EspBootPosition {
        step: BOARD_STOP,
        pc: ASSIST_READ_PC,
    };
    assert_eq!(board.esp_boot_position(), Some(stop));
    let mut qemu_tail: Vec<_> = pcs.by_ref().take(QEMU_STOP - step).collect();
    assert_eq!(qemu_tail.last(), Some(&(QEMU_STOP, ASSIST_READ_PC)));
    board_tail.push(ASSIST_READ_PC);

    // ROM ets_write_char: 0x400481e0 — ret (8082), перехода на себя нет.
    // В трассе QEMU -d exec встречаются две соседние записи одного блока ret.
    qemu_tail.dedup_by(|(_, pc), (_, prev)| *pc == 0x4004_81e0 && *pc == *prev);
    let (board_tail, board_usb_calls) = без_внутренних_pc_usb(board_tail);
    let (qemu_tail, qemu_usb_calls) =
        без_внутренних_pc_usb(qemu_tail.into_iter().map(|(_, pc)| pc));
    assert_eq!((board_usb_calls, qemu_usb_calls), (99, 99));
    assert_eq!(board_tail.len(), 8701); // 8602 внешних PC и 99 входов в USB.
    assert_eq!(board_tail.len(), qemu_tail.len());
    for (index, (&board_pc, qemu_pc)) in board_tail.iter().zip(qemu_tail).enumerate() {
        assert_eq!(board_pc, qemu_pc, "PC после FIFO, индекс {index}");
    }

    let banner = uart.split_inclusive('\n').next().unwrap();
    assert_eq!(banner, "ESP-ROM:esp32c3-api1-20210207\r\n");
    assert_eq!(board.uart_take(), banner.as_bytes());

    // Точки из nm загрузчика и двух трасс этого образа, номера шагов от единицы.
    const LOADER_ENTRY: u32 = 0x403c_bf1a;
    const LOADER_STEP: u64 = 265349;
    const QEMU_LOADER_STEP: usize = 519246;
    const REGI2C_PC: u32 = 0x403c_ca60;
    const BOARD_LAST: u64 = 504898;
    const QEMU_LAST: usize = 519764;
    let mut board_suffix = Vec::new();
    loop {
        let position = board.esp_boot_position().unwrap();
        if position.step + 1 == LOADER_STEP {
            assert_eq!(position.pc, LOADER_ENTRY);
        }
        board_suffix.push(position.pc);
        if position.step + 1 == BOARD_LAST {
            break;
        }
        assert!(position.step < BOARD_LAST);
        assert_eq!(board.run_cycles(1).unwrap().cycle, position.step + 1);
    }
    let regi2c_stop = EspBootPosition {
        step: BOARD_LAST - 1,
        pc: REGI2C_PC,
    };
    assert_eq!(board.esp_boot_position(), Some(regi2c_stop));
    for _ in 0..2 {
        assert_eq!(
            board.run_cycles(1),
            Err(regi2c_reset_fault(REGI2C_PC, ANA_CONFIG))
        );
        assert_eq!(board.esp_boot_position(), Some(regi2c_stop));
    }
    let mut qemu_suffix: Vec<_> = std::iter::once(ASSIST_READ_PC)
        .chain(
            pcs.by_ref()
                .take(QEMU_LAST - QEMU_STOP - 1)
                .map(|(_, pc)| pc),
        )
        .collect();
    assert_eq!(qemu_suffix[QEMU_LOADER_STEP - QEMU_STOP - 1], LOADER_ENTRY);
    assert_eq!(qemu_suffix.last(), Some(&REGI2C_PC));
    // 0x4004a198 — c.j на 0x4004a166 по esp32c3_rev3_rom.elf.
    // Соседнее повторение того же c.j в логе QEMU не является новой инструкцией.
    qemu_suffix.dedup_by(|pc, prev| matches!(*pc, 0x4004_81e0 | 0x4004_a198) && pc == prev);
    let (board_suffix, board_usb) = без_внутренних_pc_usb(board_suffix);
    let (qemu_suffix, qemu_usb) = без_внутренних_pc_usb(qemu_suffix);
    assert_eq!(board_usb, qemu_usb);
    let board_suffix = без_повторов_ожидания_uart(&board_suffix);
    let qemu_suffix = без_повторов_ожидания_uart(&qemu_suffix);
    assert_eq!(board_suffix.len(), 117761);
    assert_eq!(board_suffix.len(), qemu_suffix.len());
    for (index, (&board_pc, qemu_pc)) in board_suffix.iter().zip(qemu_suffix).enumerate() {
        assert_eq!(board_pc, qemu_pc, "PC после ASSIST_DEBUG, индекс {index}");
    }

    let entry_line = format!("entry {LOADER_ENTRY:#010x}\r\n");
    let (rom_output, _) = uart
        .split_once(&entry_line)
        .expect("UART QEMU до входа во вторую ступень");
    let mut board_uart = banner.as_bytes().to_vec();
    board_uart.extend(board.uart_take());
    assert_eq!(
        board_uart,
        [rom_output.as_bytes(), entry_line.as_bytes()].concat()
    );
    assert_eq!(board_uart.len(), 236);

    let app_main = std::env::var("SIM_ESP_BOOT_PC_APP_MAIN").unwrap();
    let app_main = u32::from_str_radix(app_main.trim().trim_start_matches("0x"), 16).unwrap();
    let (app_index, _) = pcs
        .find(|&(_, pc)| pc == app_main)
        .expect("QEMU вошёл в app_main");
    assert_eq!(app_index + 1, 2403358); // Потактовая трасса QEMU 9.2.2 этого образа.
    println!(
        "Совпали PC: {MATCHED} + {joined} + {after_usb} до FIFO, {} после FIFO и {} от прежней остановки до REGI2C, с исключением тел USB, повторов ожидания UART и дублей трассы. UART0 ROM {} байт совпали. BoardSim: вход во вторую ступень #{LOADER_STEP}, отказ #{BOARD_LAST} PC {REGI2C_PC:#010x}. QEMU: отказной PC #{QEMU_LAST}, app_main {app_main:#010x} #{}.",
        board_tail.len(),
        board_suffix.len(),
        board_uart.len(),
        app_index + 1,
    );
}

fn qemu_pcs() -> impl Iterator<Item = u32> {
    let trace = std::fs::File::open(std::env::var("SIM_ESP_BOOT_QEMU_PC").unwrap()).unwrap();
    BufReader::new(trace)
        .lines()
        .map(Result::unwrap)
        .filter(|line| line.starts_with("Trace "))
        .map(|line| {
            let fields = line.split_once('[').unwrap().1;
            u32::from_str_radix(fields.split('/').nth(1).unwrap(), 16).unwrap()
        })
}

fn без_внутренних_pc_usb(pcs: impl IntoIterator<Item = u32>) -> (Vec<u32>, usize) {
    // esp32c3_rev3_rom.elf, usb_uart_tx_one_char: от входа до ret
    // короткого пути без хоста или ret после записи/ожидания EP1.
    const ENTER: u32 = 0x4004_c8e2;
    const RETURN_WITHOUT_HOST: u32 = 0x4004_c900;
    const RETURN_AFTER_WAIT: u32 = 0x4004_c958;
    let mut outside_usb = Vec::new();
    let mut calls = 0;
    let mut in_usb = false;
    for pc in pcs {
        match pc {
            ENTER => {
                assert!(!in_usb);
                in_usb = true;
                outside_usb.push(pc);
            }
            RETURN_WITHOUT_HOST | RETURN_AFTER_WAIT => {
                assert!(in_usb);
                in_usb = false;
                calls += 1;
            }
            _ if !in_usb => outside_usb.push(pc),
            _ => (),
        }
    }
    assert!(!in_usb);
    (outside_usb, calls)
}

fn без_повторов_ожидания_uart(mut pcs: &[u32]) -> Vec<u32> {
    // ROM rev. 3: uart_tx_one_char_uart, uart_tx_flush, uart_tx_wait_idle.
    // Первое чтение UART_STATUS/FSM_STATUS остаётся. QEMU передаёт байт
    // в chardev сразу, BoardSim ждёт истечения времени UART-кадров.
    const POLLS: [&[u32]; 3] = [
        &[0x4004_c0d4, 0x4004_c0d6, 0x4004_c0d8, 0x4004_c0dc],
        &[0x4004_c48c, 0x4004_c48e, 0x4004_c490],
        &[0x4004_c4c6, 0x4004_c4c8, 0x4004_c4ca, 0x4004_c4cc],
    ];
    let mut output = Vec::new();
    while let Some((&pc, rest)) = pcs.split_first() {
        if let Some(poll) = POLLS.into_iter().find(|poll| pcs.starts_with(poll)) {
            output.extend_from_slice(poll);
            pcs = &pcs[poll.len()..];
            while let Some(rest) = pcs.strip_prefix(poll) {
                pcs = rest;
            }
        } else {
            output.push(pc);
            pcs = rest;
        }
    }
    output
}

// ESP-IDF regi2c_defs.h:12–24, clk_tree_ll.h:289–404 и system_reg.h.
const ANA_CONF0: u32 = 0x6000_e040;
const ANA_CONFIG: u32 = 0x6000_e044;
const ANA_CONFIG2: u32 = 0x6000_e048;
const STOP_HIGH: u32 = 1 << 2;
const STOP_LOW: u32 = 1 << 3;
const CPU_PER: u32 = 0x600c_0008;
const CPU_PLL: u32 = 1 << 10;
const PLL_480: u32 = 1 << 2;
const CPU_160: u32 = 1;
const CPU_WAIT_FORCE_ON: u32 = 1 << 3;
// PAC ESP32-C3 i2c_ana_mst/ana_conf0.rs:27–30.
const CAL_DONE: u32 = 1 << 24;
// esp32c3_rev3_rom.elf, rom_chip_i2c_{readReg_org,writeReg}.
const REGI2C: u32 = 0x6000_e000;
const REGI2C_READ: u32 = 1 << 26;
const REGI2C_WRITE: u32 = REGI2C_READ | (1 << 24);
// regi2c_bbpll.h:18, RTC CNTL TRM регистр 9.1 и rtc_cntl_reg.h:101–118.
const BBPLL: u32 = 0x66;
const RTC_OPTIONS: u32 = 0x6000_8000;
const RTC_OPTIONS_RESET: u32 = (0b111 << 26) | (2 << 14) | (1 << 13);
const PLL_POWER_OFF: u32 = (1 << 10) | (1 << 8);

fn regi2c_reset_fault(pc: u32, addr: u32) -> BoardFault {
    BoardFault::UnsupportedRegister {
        pc,
        addr,
        write: false,
        size: 4,
        block: "RTC REGI2C: состояние холодного сброса не задокументировано",
    }
}

const RESET_PROFILE: &str =
    include_str!("../firmware-tests/esp_sdk/reference/esp32c3-qemu-reset.toml");

#[test]
fn профиль_сброса_проверяет_адрес_поле_значение_и_источник() {
    let valid = RESET_PROFILE.split("[[reset]]").nth(1).unwrap();
    let valid = format!("[[reset]]{valid}");
    assert!(EspResetProfile::parse(&valid).is_ok());
    for input in [
        valid.replace("0x6000e044", "0x6000e045"),
        valid.replace("ANA_CONFIG_REG", "неизвестное поле"),
        valid.replace("0x00ffffff", "inf"),
        valid.replace("0x00ffffff", "nan"),
        valid.replace("0x00ffffff", "0x100000000"),
        valid.replace("\"qemu\"", "\"нет источника такого вида\""),
        valid.replace("source = ", "unknown = "),
        format!("{valid}\nvalue = 0"),
        format!("{valid}{valid}"),
    ] {
        assert!(
            matches!(
                EspResetProfile::parse(&input),
                Err(BoardFault::Misconfigured(_))
            ),
            "{input}"
        );
    }
}

#[test]
fn профиль_сброса_считает_только_подстановки_и_сохраняет_их_в_снимке() {
    let (entry, _) = RESET_PROFILE.split_once("kind = \"qemu\"").unwrap();
    let profile = EspResetProfile::parse(&format!("{entry}kind = \"qemu\"")).unwrap();
    let mut probe = Probe::default();
    probe.expect_read(ANA_CONFIG, 0x00ff_ffff); // QEMU esp32c3.c:136–139.
    probe.expect_read(ANA_CONFIG, 0x00ff_ffff);
    // Запись наблюдаемого слова, после которой профиль больше не нужен.
    let written = 0;
    probe.put(ANA_CONFIG, written);
    probe.expect_read(ANA_CONFIG, written);
    let unsupported = probe.read(ANA_CONFIG2);
    let mut board = probe_board(&probe, None, &[EBREAK], &[EBREAK], Some(profile));
    board.run_cycles(3).unwrap(); // LUI, ADDI и первое чтение.
    let [used] = board.esp_reset_value_uses() else {
        panic!("нужно одно допущение")
    };
    assert_eq!(
        (used.entry.address, used.entry.kind, used.reads),
        (ANA_CONFIG, EspResetValueKind::Qemu, 1)
    );
    let snapshot = board.clone_state();
    assert_eq!(
        board.run_cycles(probe.0.len() as u64),
        Err(regi2c_reset_fault(unsupported, ANA_CONFIG2))
    );
    assert_eq!(snapshot.esp_reset_value_uses()[0].reads, 1);
    assert_eq!(board.esp_reset_value_uses()[0].reads, 2);
    for on in [false, true] {
        board
            .apply(InputEvent {
                cycle: board.cycles(),
                input: BoardInput::Power { on },
            })
            .unwrap();
        assert_eq!(board.esp_reset_value_uses()[0].reads, 2);
    }
    board.run_cycles(3).unwrap();
    assert_eq!(board.esp_reset_value_uses()[0].reads, 3);
}

#[test]
fn профиль_сброса_даёт_только_биты_остановки_bbpll() {
    let profile = EspResetProfile::parse(RESET_PROFILE).unwrap();
    let mut probe = Probe::default();
    probe.expect_read(ANA_CONF0, STOP_HIGH | STOP_LOW);
    probe.put(ANA_CONF0, STOP_LOW);
    probe.expect_read(ANA_CONF0, STOP_LOW);
    let end = probe.pc();
    probe.0.push(EBREAK);
    let mut board = probe_board(&probe, None, &[EBREAK], &[EBREAK], Some(profile));
    assert_eq!(
        board.run_cycles(probe.0.len() as u64),
        Err(BoardFault::Breakpoint { pc: end })
    );
    let [used] = board.esp_reset_value_uses() else {
        panic!("нужно одно допущение")
    };
    assert_eq!(
        (used.entry.address, used.entry.value, used.reads),
        (ANA_CONF0, STOP_HIGH | STOP_LOW, 1)
    );
}

#[test]
fn профиль_сброса_даёт_ana_config2_до_первой_записи() {
    let profile = EspResetProfile::parse(RESET_PROFILE).unwrap();
    let mut probe = Probe::default();
    // У QEMU чтение после сброса и после записи одинаково; модель хранит запись.
    probe.expect_read(ANA_CONFIG2, 0x00ff_ffff); // QEMU esp32c3.c:136–139.
    probe.put(ANA_CONFIG2, 0);
    probe.expect_read(ANA_CONFIG2, 0);
    let end = probe.pc();
    probe.0.push(EBREAK);
    let mut board = probe_board(&probe, None, &[EBREAK], &[EBREAK], Some(profile));
    assert_eq!(
        board.run_cycles(probe.0.len() as u64),
        Err(BoardFault::Breakpoint { pc: end })
    );
    let [used] = board.esp_reset_value_uses() else {
        panic!("нужно одно допущение")
    };
    assert_eq!((used.entry.address, used.reads), (ANA_CONFIG2, 1));
}

#[test]
fn профиль_сброса_подставляет_слово_master0_до_команды() {
    let profile = EspResetProfile::parse(RESET_PROFILE).unwrap();
    let mut probe = Probe::default();
    probe.expect_read(REGI2C, 0x00ff_ffff); // QEMU esp32c3.c:136–139.
    // rom_chip_i2c_writeReg: START, WRITE, DATA=0, регистр 7, ведомый SAR 0x69.
    let command = 0x0500_0769;
    probe.put(REGI2C, command);
    probe.expect_read(REGI2C, command);
    let end = probe.pc();
    probe.0.push(EBREAK);
    let mut board = probe_board(&probe, None, &[EBREAK], &[EBREAK], Some(profile));
    assert_eq!(
        board.run_cycles(probe.0.len() as u64),
        Err(BoardFault::Breakpoint { pc: end })
    );
    let [used] = board.esp_reset_value_uses() else {
        panic!("нужно одно допущение")
    };
    assert_eq!((used.entry.address, used.reads), (REGI2C, 1));
}

#[test]
fn профиль_сброса_подставляет_данные_ведомых_при_rmw() {
    let mut probe = Probe::default();
    // Адреса из regi2c_{bbpll,saradc,dig_reg}.h, маски из таблицы ROM 0x3ff1a134.
    for (address, select_bit) in [
        (0x0566, 17),
        (0x0966, 17),
        (0x0769, 18),
        (0x0d6d, 14),
        (0x0061, 22),
    ] {
        probe.put(ANA_CONFIG, !(1 << select_bit));
        let read = 0x0400_0000 | address; // rom_chip_i2c_readReg_org: START.
        probe.put(REGI2C, read);
        probe.expect_read(REGI2C, read | 0x00ff_0000); // DATA QEMU из слова MASTER0.
        probe.put(REGI2C, 0x0500_0000 | address); // START | WRITE, DATA=0.
        probe.put(REGI2C, read);
        probe.expect_read(REGI2C, read);
    }
    let end = probe.pc();
    probe.0.push(EBREAK);
    let profile = EspResetProfile::parse(RESET_PROFILE).unwrap();
    let mut board = probe_board(&probe, None, &[EBREAK], &[EBREAK], Some(profile));
    assert_eq!(
        board.run_cycles(probe.0.len() as u64),
        Err(BoardFault::Breakpoint { pc: end })
    );
    let fields: Vec<_> = board
        .esp_reset_value_uses()
        .iter()
        .map(|used| (used.entry.field.as_str(), used.reads))
        .collect();
    assert_eq!(
        fields,
        [
            ("BBPLL.5", 1),
            ("BBPLL.9", 1),
            ("DIG_REG.13", 1),
            ("SARADC.7", 1),
            ("ULP_BOD.0", 1)
        ]
    );
}

#[test]
fn профиль_sfdp_заполняет_только_известные_байты() {
    let profile = EspResetProfile::parse(RESET_PROFILE).unwrap();
    for (offset, bits, fault) in [
        (0x10, 8, None),
        (0x10, 16, Some("не задан байт flash SFDP.0x11")),
        (0x11, 8, Some("не задан байт flash SFDP.0x11")),
    ] {
        // spi_mem_reg.h и bootloader_flash_read_sfdp: 0x5a, 24 бита адреса, 8 dummy.
        let mut probe = Probe::default();
        probe.put(SPI1 + 4, offset);
        probe.put(SPI1 + 0x18, 0xf000_0000);
        probe.put(SPI1 + 0x1c, 23 << 26 | 7);
        probe.put(SPI1 + 0x20, 7 << 28 | 0x5a);
        probe.put(SPI1 + 0x28, bits - 1);
        probe.put(SPI1 + 0x58, 0x1122_3344);
        probe.put(SPI1, 1 << 18);
        probe.expect_read(SPI1 + 0x58, 0x1122_3300);
        let end = probe.pc();
        probe.0.push(EBREAK);
        let mut board = probe_board(&probe, None, &[EBREAK], &[EBREAK], Some(profile.clone()));
        let result = board.run_cycles(probe.0.len() as u64);
        if let Some(detail) = fault {
            assert_eq!(result, Err(BoardFault::Unsupported(detail.into())));
            assert!(board.esp_reset_value_uses().is_empty());
        } else {
            assert_eq!(result, Err(BoardFault::Breakpoint { pc: end }));
            let [used] = board.esp_reset_value_uses() else {
                panic!("нужен байт SFDP")
            };
            assert_eq!(
                (used.entry.address, used.entry.field.as_str(), used.reads),
                (SPI1 + 0x58, "SFDP.0x10", 1)
            );
        }
    }
}

#[test]
fn профиль_sram1_не_затирает_записи_и_не_считает_оборванное_чтение() {
    // TRM §3.3.2: алиасы IBUS/DBUS на один и тот же SRAM1.
    const DRAM: u32 = 0x3fc8_1000;
    let profile = EspResetProfile::parse(
        r#"[[reset]]
address = 0x3fc81000
field = "SRAM1_COLD_WORD"
value = 0
source = "https://github.com/espressif/qemu/blob/esp-develop-9.2.2-20260417/util/mmap-alloc.c#L171-L193"
kind = "qemu""#,
    )
    .unwrap();
    let mut probe = Probe::default();
    probe.li(5, DRAM + 1);
    probe.li(6, 0xe1);
    probe.0.push(0x0062_8023); // sb t1,0(t0).
    probe.expect_read(SRAM + 0x1000, 0x0000_e100);
    probe.put(DRAM, 0x89ab_cdef);
    probe.expect_read(DRAM, 0x89ab_cdef);
    probe.read(DRAM + 3);
    let mut board = probe_board(&probe, None, &[EBREAK], &[EBREAK], Some(profile.clone()));
    assert_eq!(
        board.run_cycles(probe.0.len() as u64),
        Err(BoardFault::Unsupported(
            "содержимое SRAM по адресу 0x3fc81003 не задано".into()
        ))
    );
    let [used] = board.esp_reset_value_uses() else {
        panic!("должно учитываться только одно успешное чтение");
    };
    assert_eq!((used.entry.address, used.reads), (DRAM, 1));
    assert_eq!(used.read_addresses, [(SRAM + 0x1000, 1)].into());
    assert!(board.capabilities().blocks_hardware_proof);

    let mut probe = Probe::default();
    probe.jump(SRAM + 0x1000, false);
    let mut board = probe_board(&probe, None, &[EBREAK], &[EBREAK], Some(profile));
    assert!(board.run_cycles(probe.0.len() as u64 + 1).is_err());
    assert!(board.esp_reset_value_uses().is_empty());
}

#[test]
fn данные_sram1_читаются_без_выравнивания_при_raw_flash() {
    // TRM §3.3.2, SRAM1 DBUS. Заголовок esp_image_header_t загружается через LHU.
    const DRAM: u32 = 0x3fc8_0000;
    let mut probe = Probe::default();
    probe.put(DRAM, 0x4433_2211);
    probe.put(DRAM + 4, 0x8877_6655);
    probe.li(5, DRAM + 3);
    probe.0.push(0x0002_d383); // lhu t2,0(t0).
    probe.li(6, 0x5544);
    probe.0.extend([0x0063_8463, EBREAK]); // beq t2,t1,+8.
    probe.expect_read(DRAM + 1, 0x5544_3322);
    let pc = probe.read(ANA_CONFIG + 1);
    let mut board = board_for(&probe, None);
    assert_eq!(
        board.run_cycles(probe.0.len() as u64),
        Err(BoardFault::Unaligned {
            pc,
            addr: ANA_CONFIG + 1,
            size: size_of::<u32>() as u8,
        })
    );
}

#[test]
fn mwdt_соблюдает_защиту_и_выдержку_от_apb() {
    // timer_group_reg.h, TIMG1. TRM рег. 11.10–11.24.
    const TIMG1: u32 = 0x6002_0000;
    const CONFIG: u32 = TIMG1 + 0x48;
    const UNLOCK: u32 = TIMG1 + 0x64;
    const STATUS: u32 = TIMG1 + 0x78;
    const WDT_INT: u32 = 1 << 1;
    let mut probe = Probe::default();
    let reset = (1 << 18) | (1 << 15) | (1 << 14);
    let active = (1 << 31) | (1 << 29);
    probe.expect_read(CONFIG, reset);
    probe.put(UNLOCK, 0);
    probe.put(CONFIG, active);
    probe.expect_read(CONFIG, reset);
    probe.put(UNLOCK, 0x50d8_3aa1);
    let prescale = 3;
    let hold = 20;
    probe.put(TIMG1 + 0x4c, prescale << 16);
    probe.put(TIMG1 + 0x50, hold);
    probe.put(TIMG1 + 0x70, WDT_INT);
    probe.put(CONFIG, active | (1 << 22));
    let begin = probe.0.len() as u64;
    probe.expect_read(STATUS, 0);
    // CPU и APB идут от XTAL/2. До первого IRQ нужно hold * prescale тактов.
    let until = begin + u64::from(prescale * hold);
    probe.0.resize(until as usize, 0x0000_0013); // RISC-V nop.
    probe.expect_read(STATUS, WDT_INT);
    probe.put(TIMG1 + 0x7c, WDT_INT);
    probe.expect_read(STATUS, 0);
    let end = probe.pc();
    probe.0.push(EBREAK);
    let mut board = board_for(&probe, None);
    assert_eq!(
        board.run_cycles(probe.0.len() as u64),
        Err(BoardFault::Breakpoint { pc: end })
    );
}

#[test]
fn iomux_сохраняет_режим_сна_без_переключения_активной_функции() {
    // io_mux_reg.h: GPIO4, MTMS при MCU_SEL=0. SLP_IE[4], MCU_WPU[8].
    const MUX4: u32 = 0x6000_9014;
    let reset = (2 << 10) | (1 << 9); // TRM табл. 5.12-1, DRV и IE.
    let mut probe = Probe::default();
    probe.expect_read(MUX4, reset);
    probe.put(MUX4, reset | (1 << 4));
    probe.expect_read(MUX4, reset | (1 << 4));
    probe.put(MUX4, reset | (1 << 8));
    let mut board = board_for(&probe, None);
    assert_eq!(
        board.run_cycles(probe.0.len() as u64),
        Err(BoardFault::Unsupported(
            "режим блока IO MUX: выбранная прямая функция".into()
        )),
    );
    assert_eq!(board.esp_boot_position().unwrap().pc, probe.pc() - 4);
}

#[test]
fn rng_без_явного_виртуального_источника_останавливает_прогон() {
    let mut probe = Probe::default();
    probe.read(0x6002_60b0); // TRM рег. 24.1.
    let mut board = board_for(&probe, None);
    assert_eq!(
        board.run_cycles(3),
        Err(BoardFault::Unsupported(
            "RNG: источник энтропии не задан".into()
        ))
    );
    assert_eq!(board.esp_virtual_rng_use(), None);
}

#[test]
fn калибратор_rtc_считает_номинальный_rc_slow_и_проверяет_источник() {
    // TRM рег. 11.18–11.20, §6.2.3. CPU=XTAL/2, RC_SLOW=136 кГц.
    const CFG: u32 = 0x6001_f068;
    let mut probe = Probe::default();
    probe.put(CFG, 0); // Останов исходного периодического режима.
    let cycles = 100; // Первая выборка из трассы QEMU.
    let start = (1 << 31) | (cycles << 16);
    probe.put(CFG, start);
    probe.expect_read(CFG, start);
    let cpu_ticks = (u64::from(cycles) * 20_000_000).div_ceil(136_000) as usize;
    probe.0.resize(probe.0.len() + cpu_ticks, 0x0000_0013); // RISC-V nop.
    probe.expect_read(CFG, start | (1 << 15));
    probe.expect_read(CFG + 4, 0x397180); // XTAL 40 МГц × 100 / 136 кГц, QEMU CFG1.
    probe.put(CFG, 0);
    probe.put(CFG, start | (1 << 13)); // RC_FAST_DIV без источника.
    let mut board = board_for(&probe, None);
    assert_eq!(
        board.run_cycles(probe.0.len() as u64),
        Err(BoardFault::Unsupported(
            "TIMG: калибровка требует RC_SLOW_CLK и ненулевого RTC_CALI_MAX".into()
        ))
    );
}

#[test]
fn regi2c_отклоняет_чтение_неизвестного_сброса() {
    for addr in [REGI2C, ANA_CONF0, ANA_CONFIG, ANA_CONFIG2] {
        let mut p = Probe::default();
        let pc = p.read(addr);
        let mut board = board_for(&p, None);
        assert_eq!(
            board.run_cycles(p.0.len() as u64),
            Err(regi2c_reset_fault(pc, addr))
        );
        assert_eq!(board.run_cycles(1), Err(regi2c_reset_fault(pc, addr)));
    }
}

#[test]
fn regi2c_ведомые_хранят_записанные_байты_и_раздельные_состояния() {
    let mut p = Probe::default();
    p.put(ANA_CONFIG2, 0x1ff40); // ROM rev. 3, слова настройки в rom_get_i2c_hostid.
    p.expect_read(ANA_CONFIG2, 0x1ff40);
    p.put(ANA_CONFIG, !(1 << 17)); // regi2c_ctrl_ll.h:25–31, открываем BBPLL.
    // Адреса и номера из regi2c_{saradc,bias,bbpll,dig_reg,brownout}.h.
    // Байты 0x31..0x35 выбраны стендом, маски — таблица ROM в 0x3ff1a134.
    let registers = [
        (0x0769, 0x31, 1 << 18),
        (0x016a, 0x32, 1 << 13),
        (0x0466, 0x33, 1 << 17),
        (0x046d, 0x34, 1 << 14),
        (0x0561, 0x35, 1 << 22),
    ];
    for (address, data, _) in registers {
        p.put(REGI2C, REGI2C_WRITE | data << 16 | address);
    }
    let checkpoint = p.0.len() as u64;
    for (address, data, mask) in registers {
        p.put(ANA_CONFIG, !mask);
        p.expect_read(ANA_CONFIG, !mask);
        p.put(REGI2C, REGI2C_READ | address);
        p.expect_read(REGI2C, REGI2C_READ | data << 16 | address);
    }
    let success = p.pc();
    p.0.push(EBREAK);
    let mut board = board_for(&p, None);
    board.run_cycles(checkpoint).unwrap();
    let mut copy = board.clone_state();
    for branch in [&mut board, &mut copy] {
        assert_eq!(
            branch.run_cycles(p.0.len() as u64),
            Err(BoardFault::Breakpoint { pc: success })
        );
    }
    board.reset();
    assert_eq!(board.esp_boot_position().unwrap().pc, ROM);
    assert_eq!(
        board.run_cycles(p.0.len() as u64),
        Err(BoardFault::Breakpoint { pc: success })
    );
}

#[test]
fn regi2c_отклоняет_неизвестный_байт_маршрут_и_питание() {
    for (config, powered, addr, cmd, detail) in [
        (!0, true, REGI2C, REGI2C_READ | 0x0769, "ведомый отключён"),
        (!0, true, REGI2C, REGI2C_WRITE | 0x0466, "ведомый отключён"),
        (
            !(1 << 18),
            true,
            REGI2C,
            REGI2C_READ | 0x0769,
            "байт ведомого не записан, сброс неизвестен",
        ),
        (
            0,
            true,
            REGI2C,
            REGI2C_WRITE | 0xff66,
            "неизвестный регистр ведомого",
        ),
        (
            0,
            true,
            REGI2C,
            REGI2C_WRITE | 0xff,
            "неизвестный ведомый MASTER0",
        ),
        (
            0,
            true,
            REGI2C,
            REGI2C_READ | 0x0866,
            "аналоговый результат калибровки неизвестен",
        ),
        (
            0,
            false,
            REGI2C,
            REGI2C_WRITE | 0x0466,
            "питание BBPLL или её интерфейса отключено",
        ),
        (
            0,
            true,
            REGI2C + 4,
            REGI2C_READ | 0x0466,
            "MASTER1 не моделируется",
        ),
    ] {
        let mut p = Probe::default();
        p.put(ANA_CONFIG, config);
        if !powered {
            p.put(RTC_OPTIONS, RTC_OPTIONS_RESET | PLL_POWER_OFF);
        }
        p.put(addr, cmd);
        let pc = p.pc() - 4;
        let mut board = board_for(&p, None);
        let err = board.run_cycles(p.0.len() as u64).unwrap_err();
        assert!(
            matches!(err, BoardFault::UnsupportedRegister { pc: at, addr: failed, write: true, size: 4, block } if at == pc && failed == addr && block.ends_with(detail)),
            "{err}"
        );
    }
}

fn program_bbpll(p: &mut Probe, mode: u32, divider: u32) {
    p.put(ANA_CONFIG, (0x3ff << 8) & !(1 << 17)); // regi2c_ctrl_ll.h:17–31.
    p.put(ANA_CONF0, STOP_LOW);
    // clk_tree_ll.h:319–395, 40 МГц. Итоговые байты после SDK RMW.
    for (reg, data) in [
        (4, mode),
        (2, 0x50),
        (3, divider),
        (5, 0),
        (6, 0x63),
        (9, 2),
    ] {
        p.put(REGI2C, REGI2C_WRITE | data << 16 | reg << 8 | BBPLL);
    }
}

#[test]
fn bbpll_два_рецепта_дают_захват_и_разрешают_только_свой_диапазон() {
    for (mode, divider, band) in [(0x69, 4, 0), (0x6b, 8, PLL_480)] {
        let mut p = Probe::default();
        p.put(ANA_CONF0, STOP_HIGH | CAL_DONE);
        p.expect_read(ANA_CONF0, STOP_HIGH); // Запись статуса не подделывает захват.
        program_bbpll(&mut p, mode, divider);
        p.expect_read(ANA_CONF0, STOP_LOW | CAL_DONE);
        p.put(CPU_PER, CPU_WAIT_FORCE_ON | band | CPU_160);
        p.put(SYSCLK, CPU_PLL);
        let switched = p.0.len() as u64;
        p.expect_read(SYSCLK, CPU_PLL);
        p.put(CPU_PER, CPU_WAIT_FORCE_ON | (band ^ PLL_480) | CPU_160);
        let wrong_band_pc = p.pc() - 4;
        let mut board = board_for(&p, None);
        board.run_cycles(switched).unwrap();
        let start = board.cycle_time_quarter_ns(switched).unwrap().unwrap();
        board.run_cycles(1).unwrap();
        let next = board.cycle_time_quarter_ns(switched + 1).unwrap().unwrap();
        // TRM §6.2.4, 160 МГц: 6,25 нс на такт.
        assert_eq!(next - start, 25);
        let err = board.run_cycles(p.0.len() as u64).unwrap_err();
        assert!(
            matches!(err, BoardFault::Unsupported(ref why) if why.contains("нет подтверждённого цифрового захвата")),
            "{err}"
        );
        assert_eq!(board.esp_boot_position().unwrap().pc, wrong_band_pc);
    }
}

#[test]
fn bbpll_несогласованный_рецепт_и_потеря_питания_блокируют_переход() {
    for power_down in [false, true] {
        let mut p = Probe::default();
        // Два случая: 480 МГц с делителем из рецепта 320 МГц и отключение
        // после захвата на правильном рецепте (clk_tree_ll.h:319–395).
        program_bbpll(&mut p, 0x6b, if power_down { 8 } else { 4 });
        if power_down {
            p.expect_read(ANA_CONF0, STOP_LOW | CAL_DONE);
            p.put(RTC_OPTIONS, RTC_OPTIONS_RESET | PLL_POWER_OFF);
            p.put(ANA_CONF0, STOP_LOW);
            p.put(RTC_OPTIONS, RTC_OPTIONS_RESET);
        }
        p.expect_read(ANA_CONF0, STOP_LOW);
        p.put(SYSCLK, CPU_PLL);
        let mut board = board_for(&p, None);
        let err = board.run_cycles(p.0.len() as u64).unwrap_err();
        assert!(
            matches!(err, BoardFault::Unsupported(ref why) if why.contains("нет подтверждённого цифрового захвата")),
            "{err}"
        );
        assert_eq!(board.esp_boot_position().unwrap().pc, p.pc() - 4);
    }
}

#[test]
fn bbpll_без_захвата_недоступна_и_нельзя_отключить_текущий_источник_cpu() {
    for configured in [false, true] {
        let mut p = Probe::default();
        if configured {
            program_bbpll(&mut p, 0x6b, 8); // CLK_LL PLL 480 МГц, XTAL 40 МГц.
        }
        p.put(SYSCLK, CPU_PLL);
        if configured {
            p.expect_read(SYSCLK, CPU_PLL);
            p.put(RTC_OPTIONS, RTC_OPTIONS_RESET | PLL_POWER_OFF);
        }
        let mut board = board_for(&p, None);
        let err = board.run_cycles(p.0.len() as u64).unwrap_err();
        let reason = if configured {
            "питание BBPLL или её интерфейса отключено"
        } else {
            "нет подтверждённого цифрового захвата"
        };
        assert!(
            matches!(err, BoardFault::Unsupported(ref why) if why.contains(reason)),
            "{err}"
        );
        assert_eq!(board.esp_boot_position().unwrap().pc, p.pc() - 4);
    }
}

#[test]
#[ignore = "нужен установленный образ ROM rev. 3 и входы штатного прогона SDK"]
fn regi2c_читается_подлинной_подпрограммой_rom() {
    // Espressif esp32c3_rev3_rom.elf, rom_chip_i2c_readReg_org:
    // a0 — ведомый, a1 — маска чтения, a2 — хост, a3 — номер регистра.
    const ROM_READ: u32 = 0x4003_8e58;
    const ROM_COMMAND_PC: u32 = 0x4003_8e78;
    const SAR: u32 = 0x69; // regi2c_saradc.h:18.
    const SAR_READ_MASK: u32 = 1 << 18; // Таблица ROM в 0x3ff1a134.
    const SAR_CONF: u32 = 7; // regi2c_saradc.h:21–75.
    const STAND_BYTE: u32 = 0x31;
    for written in [false, true] {
        let mut p = Probe::default();
        if written {
            p.put(
                REGI2C,
                REGI2C_WRITE | STAND_BYTE << 16 | SAR_CONF << 8 | SAR,
            );
        }
        p.li(10, SAR);
        p.li(11, SAR_READ_MASK);
        p.li(12, 0); // rom_get_i2c_hostid для SAR.
        p.li(13, SAR_CONF);
        p.jump(ROM_READ, true);
        p.li(6, STAND_BYTE);
        p.0.extend([0x0065_0463, EBREAK]); // beq a0,t1,+8.
        let success = p.pc();
        p.0.push(EBREAK);

        // Только стенд заменяет стартовые инструкции. Подпрограмма из
        // образа Espressif выполняется целиком, включая BUSY и сдвиг DATA.
        let mut rom = read_fixture("SIM_ESP_BOOT_ROM");
        let bytes = p.image();
        let len = size_of_val(p.0.as_slice());
        rom[..len].copy_from_slice(&bytes[..len]);
        let mut board = qemu_board(&rom);
        // Дизассемблирование ROM rev. 3: 19 инструкций без повтора BUSY.
        let result = board.run_cycles(p.0.len() as u64 + 19);
        if written {
            assert_eq!(result, Err(BoardFault::Breakpoint { pc: success }));
        } else {
            assert_eq!(
                result,
                Err(BoardFault::UnsupportedRegister {
                    pc: ROM_COMMAND_PC,
                    addr: REGI2C,
                    size: 4,
                    write: true,
                    block: "RTC REGI2C: байт ведомого не записан, сброс неизвестен",
                })
            );
        }
    }
}

#[test]
fn rtc_хранит_маршрут_и_задержку_brownout_и_отклоняет_запись_det() {
    // rtc_cntl_reg.h: BROWN_OUT_REG, сброс BROWN_OUT_ENA, RST_WAIT и INT_WAIT.
    const BROWN_OUT: u32 = 0x6000_80d8;
    const BROWN_OUT_RESET: u32 = (1 << 30) | (0x3ff << 16) | (1 << 4);
    const ANA_RST: u32 = 1 << 28;
    let mut p = Probe::default();
    p.expect_read(BROWN_OUT, BROWN_OUT_RESET);
    p.put(BROWN_OUT, BROWN_OUT_RESET | ANA_RST);
    p.expect_read(BROWN_OUT, BROWN_OUT_RESET | ANA_RST);
    let cfg = (BROWN_OUT_RESET & !(1 << 4)) | ANA_RST | (2 << 4); // INT_WAIT=2.
    p.put(BROWN_OUT, cfg | (1 << 29)); // CNT_CLR, WO.
    p.expect_read(BROWN_OUT, cfg);
    p.put(0x6000_8040, 1 << 9); // RTC_CNTL_INT_ENA_REG, BROWN_OUT.
    p.expect_read(0x6000_8040, 1 << 9);
    p.put(BROWN_OUT, cfg | (1 << 31)); // DET, RO.
    let mut board = board_for(&p, None);
    assert_eq!(
        board.run_cycles(p.0.len() as u64),
        Err(BoardFault::Unsupported(
            "RTC_CNTL ESP32-C3: запись зарезервированных или RO битов BROWN_OUT".into()
        )),
    );
    assert_eq!(board.esp_boot_position().unwrap().pc, p.pc() - 4);
}

// ESP-IDF soc/esp32c3/register/soc/{system,assist_debug}_reg.h.
const CPU_CLOCK_GATE: u32 = 0x600c_0000;
const CPU_RESET_GATE: u32 = 0x600c_0004;
const DEBUG_GATE: u32 = 1 << 6;
const DEDICATED_RESET: u32 = 1 << 7;
const ASSIST_DEBUG: u32 = 0x600c_e000;

fn start_assist(p: &mut Probe) {
    p.put(CPU_RESET_GATE, DEDICATED_RESET);
    p.put(CPU_CLOCK_GATE, DEBUG_GATE);
}

#[test]
fn assist_debug_различает_границы_sp_маску_irq_и_записывает_pc() {
    // TRM гл. 17 и esp32c3/hal/assist_debug_ll.h: RAW и IRQ независимы от RLS.
    const MIN: u32 = SRAM;
    const MAX: u32 = SRAM + 16; // Интервал стенда, обе границы допустимы.
    const SP_MIN: u32 = 1 << 8;
    const SP_MAX: u32 = 1 << 9;
    // interrupt_core0_reg.h, interrupts.h: ASSIST_DEBUG — источник 54.
    const MATRIX: u32 = 0x600c_2000;
    const IRQ_MAP: u32 = MATRIX + 54 * 4;
    const IRQ_ENABLE: u32 = MATRIX + 0x104;
    const IRQ_PENDING: u32 = MATRIX + 0x110;
    const LINE: u32 = 3; // Линия стенда.
    const IRQ_PRIORITY: u32 = MATRIX + 0x114 + LINE * 4;
    const IRQ_MASK: u32 = 1 << LINE;
    for (sp, raw) in [(MIN - 1, SP_MIN), (MIN, 0), (MAX, 0), (MAX + 1, SP_MAX)] {
        let mut p = Probe::default();
        p.expect_read(CPU_RESET_GATE, DEDICATED_RESET | DEBUG_GATE);
        p.expect_read(CPU_CLOCK_GATE, 0);
        start_assist(&mut p);
        p.put(IRQ_MAP, LINE);
        p.put(IRQ_ENABLE, IRQ_MASK);
        p.put(IRQ_PRIORITY, 1);
        p.put(ASSIST_DEBUG + 0x38, MIN);
        p.put(ASSIST_DEBUG + 0x3c, MAX);
        p.put(ASSIST_DEBUG + 0x44, 3); // RCD_RECORDEN | RCD_PDEBUGEN.
        p.li(2, sp);
        let record_load = p.pc() + 8; // PC инструкции после двух слов li.
        p.expect_read(ASSIST_DEBUG + 0x48, record_load - 4);
        p.expect_read(ASSIST_DEBUG + 0x4c, sp);
        p.put(ASSIST_DEBUG, SP_MIN | SP_MAX); // INTR_ENA.
        p.expect_read(ASSIST_DEBUG + 4, raw);
        p.expect_read(IRQ_PENDING, 0);
        p.put(ASSIST_DEBUG + 8, SP_MIN | SP_MAX); // INTR_RLS.
        p.expect_read(IRQ_PENDING, if raw == 0 { 0 } else { IRQ_MASK });
        p.li(2, MIN);
        p.expect_read(ASSIST_DEBUG + 4, 0);
        p.expect_read(IRQ_PENDING, 0);
        p.put(CPU_RESET_GATE, DEDICATED_RESET | DEBUG_GATE);
        p.put(CPU_RESET_GATE, DEDICATED_RESET);
        p.expect_read(ASSIST_DEBUG + 0x3c, u32::MAX); // SP_MAX reset.
        p.expect_read(ASSIST_DEBUG + 0x48, 0);
        let success = p.pc();
        p.0.push(EBREAK);
        let mut board = board_for(&p, None);
        assert_eq!(
            board.run_cycles(p.0.len() as u64),
            Err(BoardFault::Breakpoint { pc: success }),
        );
    }
}

#[test]
fn assist_debug_отклоняет_доступ_при_сбросе_и_незнакомые_поля() {
    for (start, addr, value) in [
        (false, ASSIST_DEBUG + 0x44, None),
        (true, ASSIST_DEBUG + 0x44, Some(1 << 2)), // Зарезервированное поле RCD_EN.
        (true, ASSIST_DEBUG, Some(1)),             // Мониторинг DRAM0_0 не реализован.
        (true, CPU_CLOCK_GATE, Some(1 << 7)),      // Отдельный блок dedicated GPIO.
    ] {
        let mut p = Probe::default();
        if start {
            start_assist(&mut p);
        }
        let pc = if let Some(bits) = value {
            p.put(addr, bits);
            p.pc() - 4
        } else {
            p.read(addr)
        };
        let mut board = board_for(&p, None);
        let fault = BoardFault::UnsupportedRegister {
            pc,
            addr,
            write: value.is_some(),
            size: 4,
            block: "ASSIST_DEBUG ESP32-C3",
        };
        assert_eq!(board.run_cycles(p.0.len() as u64), Err(fault.clone()));
        assert_eq!(board.run_cycles(1), Err(fault));
    }
}

// ESP-IDF soc/esp32c3/register/soc/{system,io_mux,spi_mem}_reg.h.
const FLASH_CLOCK: u32 = 0x600c_0010;
const FLASH_RESET: u32 = 0x600c_0018;
const SPI01: u32 = 1 << 1;
const GPIO15_MUX: u32 = 0x6000_9040;
const SPI_CTRL2: u32 = SPI1 + 0x10;
const SPI_CLOCK: u32 = SPI1 + 0x14;
const SPI0_FSM: u32 = SPI1 + 0x1000 + 0x54;
const SPI_SUS_STATUS: u32 = SPI1 + 0xa4;
const SPI_W0: u32 = SPI1 + 0x58;

#[test]
fn flash_сбрасывает_spi0_spi1_и_сохраняет_настройку_вывода() {
    const TIMING: u32 = (2 << 5) | 1; // Уставка стенда: CS_HOLD_TIME=2, CS_SETUP_TIME=1.
    const RESET_FSM: u32 = 1 << 31; // SPI_MEM_SYNC_RESET.
    const PAD: u32 = (2 << 10) | (1 << 9) | (1 << 8); // DRV=2, IE и WPU.
    let mut p = Probe::default();
    p.put(GPIO15_MUX, PAD); // FUNC_SPICLK_SPICLK=0 из io_mux_reg.h.
    p.expect_read(GPIO15_MUX, PAD);
    p.put(SPI_CTRL2, TIMING | RESET_FSM);
    p.expect_read(SPI_CTRL2, TIMING);
    p.put(SPI_CLOCK, 1 << 31); // esp_rom_spiflash_config_clk(1, 1), ROM rev. 3.
    p.expect_read(SPI_CLOCK, 1 << 31);
    p.expect_read(SPI0_FSM, 4 << 7); // CSPI_LOCK_DELAY=4, EM_ST/CSPI_ST=idle.
    p.expect_read(SPI_SUS_STATUS, 0);
    p.put(SPI_SUS_STATUS, 0);
    p.put(SPI_W0, u32::MAX);
    p.expect_read(SPI_W0, u32::MAX);
    p.put(FLASH_RESET, SPI01);
    p.expect_read(FLASH_RESET, SPI01);
    p.put(FLASH_RESET, 0);
    p.expect_read(SPI_W0, 0);
    p.expect_read(SPI_CTRL2, (1 << 5) | 1); // Оба поля CTRL2 сброшены в 1.
    let success = p.pc();
    p.0.push(EBREAK);
    let mut board = board_for(&p, None);
    assert_eq!(
        board.run_cycles(p.0.len() as u64),
        Err(BoardFault::Breakpoint { pc: success }),
    );
    assert_eq!(board.gpio_mode(15), None); // Нет модели уровней физической шины MSPI.
}

#[test]
fn flash_отклоняет_выключенный_spi01_и_suspend() {
    const SYS_CLOCK_RESET: u32 = 0xf9c1_e06f; // Сброс SYSTEM_PERIP_CLK_EN0.
    for control in [
        Some((FLASH_CLOCK, SYS_CLOCK_RESET & !SPI01)),
        Some((FLASH_RESET, SPI01)),
        None,
    ] {
        let mut p = Probe::default();
        let (addr, pc) = if let Some((reg, value)) = control {
            p.put(reg, value);
            (SPI_W0, p.read(SPI_W0))
        } else {
            p.put(SPI_SUS_STATUS, 1); // SPI_MEM_FLASH_SUS.
            (SPI_SUS_STATUS, p.pc() - 4)
        };
        let mut board = board_for(&p, None);
        assert_eq!(
            board.run_cycles(p.0.len() as u64),
            Err(BoardFault::UnsupportedRegister {
                pc,
                addr,
                write: control.is_none(),
                size: 4,
                block: "SPI1 flash",
            }),
        );
    }
}

// ESP-IDF soc/esp32c3/register/soc/{system,extmem}_reg.h.
const CACHE_CLOCK_RESET: u32 = 0x600c_0040;
const PRELOAD: u32 = CACHE_ENABLE + 0x34;
const AUTOLOAD: u32 = CACHE_ENABLE + 0x40;
const ILG_CLEAR: u32 = CACHE_ENABLE + 0x7c;
const WRAP: u32 = CACHE_ENABLE + 0xa8;
const OWNER: u32 = CACHE_ENABLE + 0xc4;
const FREEZE: u32 = CACHE_ENABLE + 0xcc;

#[test]
fn extmem_инициализация_и_заморозка_отключённого_кэша() {
    let mut p = Probe::default();
    p.expect_read(CACHE_CLOCK_RESET, 5); // ICACHE_CLK_ON и DCACHE_CLK_ON.
    p.put(CACHE_CLOCK_RESET, 7); // Плюс ICACHE_RESET.
    p.expect_read(CACHE_CLOCK_RESET, 7);
    p.put(CACHE_CLOCK_RESET, 5);
    p.expect_read(OWNER, 0); // EXTMEM_CACHE_MMU_OWNER, сброс 0.
    p.put(OWNER, 0xf); // IBUS и DBUS.
    p.expect_read(OWNER, 0xf);
    p.put(ILG_CLEAR, u32::MAX); // Сброс WOD-полей из Cache_MMU_Init в ROM rev. 3.
    p.put(WRAP, 0);
    p.expect_read(WRAP, 0);
    p.expect_read(PRELOAD, 1 << 1); // PRELOAD_DONE.
    p.expect_read(AUTOLOAD, 1 << 3); // AUTOLOAD_DONE.
    p.expect_read(FREEZE, 0);
    p.put(FREEZE, 1);
    p.expect_read(FREEZE, 5); // FREEZE_DONE и FREEZE_ENA.
    p.put(FREEZE, 4); // RMW снимает FREEZE_ENA и сохраняет прочитанный RO-бит.
    p.expect_read(FREEZE, 0);
    let success = p.pc();
    p.0.push(EBREAK);
    let mut board = board_for(&p, None);
    assert_eq!(
        board.run_cycles(p.0.len() as u64),
        Err(BoardFault::Breakpoint { pc: success }),
    );
}

#[test]
fn extmem_отклоняет_незнакомые_режимы() {
    for (addr, bits) in [
        (CACHE_CLOCK_RESET, 1 << 4), // Зарезервированное поле.
        (OWNER, 1),                  // Частичное назначение MMU.
        (PRELOAD, 1),                // PRELOAD_ENA.
        (AUTOLOAD, 1 << 2),          // AUTOLOAD_ENA.
        (ILG_CLEAR, 1 << 4),         // Зарезервированное поле.
        (WRAP, 1),                   // CACHE_FLASH_WRAP_AROUND.
        (FREEZE, 0b11),              // FREEZE_MODE=1: аппаратный ответ hit при промахе.
    ] {
        let mut p = Probe::default();
        p.put(addr, bits);
        let mut board = board_for(&p, None);
        assert_eq!(
            board.run_cycles(p.0.len() as u64),
            Err(BoardFault::UnsupportedRegister {
                pc: p.pc() - 4,
                addr,
                write: true,
                size: 4,
                block: "cache/MMU flash",
            }),
        );
    }
}

// ESP-IDF esp32c3/{include/soc/hwcrypto_reg.h,register/soc/system_reg.h}.
const SHA_GATE_REG: u32 = 0x600c_0014;
const SHA_RESET_REG: u32 = 0x600c_001c;
const SHA_RESET_WORD: u32 = 0x1fe;
const SHA_GATE_BIT: u32 = 1 << 2;
const SHA_ALL_RESETS: u32 = SHA_GATE_BIT | (1 << 4) | (1 << 5); // SHA/DS/HMAC, sha_ll.h.
const SHA_MODE: u32 = 0x6003_b000;
const SHA_START: u32 = SHA_MODE + 0x10;
const SHA_NEXT: u32 = SHA_MODE + 0x14;
const SHA_BUSY: u32 = SHA_MODE + 0x18;
const SHA_H: u32 = SHA_MODE + 0x40;
const SHA_TEXT: u32 = SHA_MODE + 0x80;
const SHA256_MODE: u32 = 2;

fn start_sha(p: &mut Probe) {
    p.put(SHA_GATE_REG, SHA_GATE_BIT);
    p.put(SHA_RESET_REG, SHA_RESET_WORD & !SHA_ALL_RESETS);
    p.put(SHA_MODE, SHA256_MODE);
}

#[test]
fn sha256_через_mmio_считает_два_блока_и_сбрасывается() {
    // NIST SHA-256 Examples: 448-bit Message, независимо проверено shasum -a 256.
    let message = b"abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";
    let hash = [
        0x248d6a61u32,
        0xd20638b8,
        0xe5c02693,
        0x0c3e6039,
        0xa33ce459,
        0x64ff2167,
        0xf6ecedd4,
        0x19db06c1,
    ];
    // FIPS 180-4 §5.1.1: паддинг делает вызывающая прошивка.
    let mut blocks = [0; 128];
    blocks[..message.len()].copy_from_slice(message);
    blocks[message.len()] = 0x80;
    blocks[120..].copy_from_slice(&(message.len() as u64 * u64::from(u8::BITS)).to_be_bytes());
    let mut p = Probe::default();
    p.expect_read(SHA_GATE_REG, 0);
    p.expect_read(SHA_RESET_REG, SHA_RESET_WORD);
    start_sha(&mut p);
    for (block, command) in blocks.chunks_exact(64).zip([SHA_START, SHA_NEXT]) {
        for (i, bytes) in block.chunks_exact(4).enumerate() {
            p.put(
                SHA_TEXT + i as u32 * 4,
                u32::from_le_bytes(bytes.try_into().unwrap()),
            );
        }
        p.put(command, 1);
        p.expect_read(SHA_BUSY, 0);
    }
    for (i, word) in hash.into_iter().enumerate() {
        p.expect_read(SHA_H + i as u32 * 4, word.swap_bytes());
    }
    p.put(SHA_RESET_REG, SHA_RESET_WORD);
    p.put(SHA_RESET_REG, SHA_RESET_WORD & !SHA_ALL_RESETS);
    p.expect_read(SHA_MODE, 0); // SHA-1 по сбросу, его обработка не реализована.
    let success = p.pc();
    p.0.push(EBREAK);
    let mut board = board_for(&p, None);
    assert_eq!(
        board.run_cycles(p.0.len() as u64),
        Err(BoardFault::Breakpoint { pc: success }),
    );
}

#[test]
fn sha_отклоняет_неполный_блок_continue_без_состояния_и_незнакомый_режим() {
    // Настройки стенда. SHA_H до первой операции ещё не содержит дайджест.
    for (enable, words, addr, value) in [
        (false, 0, SHA_MODE, None),
        (true, 15, SHA_START, Some(1)),
        (true, 16, SHA_NEXT, Some(1)),
        (true, 0, SHA_H, None),
        (true, 0, SHA_MODE, Some(3)),
    ] {
        let mut p = Probe::default();
        if enable {
            start_sha(&mut p);
            for i in 0..words {
                p.put(SHA_TEXT + i * 4, 0);
            }
        }
        let pc = if let Some(bits) = value {
            p.put(addr, bits);
            p.pc() - 4
        } else {
            p.read(addr)
        };
        let mut board = board_for(&p, None);
        assert_eq!(
            board.run_cycles(p.0.len() as u64),
            Err(BoardFault::UnsupportedRegister {
                pc,
                addr,
                write: value.is_some(),
                size: 4,
                block: "SHA ESP32-C3",
            }),
        );
    }
}

fn read_fixture(key: &str) -> Vec<u8> {
    std::fs::read(std::env::var(key).expect(key)).expect(key)
}

fn qemu_rom_board() -> Box<dyn BoardSim> {
    qemu_board(&read_fixture("SIM_ESP_BOOT_ROM"))
}

fn qemu_board(rom: &[u8]) -> Box<dyn BoardSim> {
    qemu_board_with_profile(rom, None)
}

fn qemu_board_with_profile(rom: &[u8], profile: Option<EspResetProfile>) -> Box<dyn BoardSim> {
    let flash = read_fixture("SIM_ESP_BOOT_FLASH");
    let fuse = read_fixture("SIM_ESP_BOOT_READBACK");
    assert_eq!(fuse.len(), EFUSE_WORDS * size_of::<u32>());
    let fuse: Vec<u32> = fuse
        .chunks_exact(size_of::<u32>())
        .map(|word| u32::from_le_bytes(word.try_into().unwrap()))
        .collect();
    let mut board = new_board(BoardId::Esp32C3);
    let options = EspBootOptions {
        systimer_phase: Some(SystimerPhase::FirstAfterTwoXtalCycles),
        // QEMU Espressif 9.2.2, hw/riscv/esp32c3.c: для 2 МиБ
        // выбирается w25x16. hw/block/m25p80.c: JEDEC и сброс RDSR.
        flash_readback: Some(EspFlashReadback {
            jedec_id: [0xef, 0x30, 0x15],
            status1: 0,
            status2: None,
        }),
    };
    let table = esp_image::DEFAULT_PARTITION_TABLE_OFFSET;
    let straps = [false, false, true]; // GPIO_STRAP_REG из монитора QEMU: 0x08.
    match profile {
        Some(profile) => board.load_esp_flash_with_profile(
            &flash,
            table,
            rom,
            &fuse,
            straps,
            // Зерно опыта — отпечаток исходных байтов flash.
            options
                .with_reset_profile(profile)
                .with_virtual_rng_seed(sim_signal::fnv1a(flash.iter().copied())),
        ),
        None => board.load_esp_flash_with_options(&flash, table, rom, &fuse, straps, options),
    }
    .unwrap();
    board
}

#[test]
#[ignore = "нужны штатный образ SDK, дамп ROM, eFuse, UART и трасса QEMU"]
fn штатный_образ_с_профилем_доходит_до_hello_world() {
    let input = std::env::var_os("SIM_ESP_BOOT_RESET_PROFILE")
        .map(|path| std::fs::read_to_string(path).unwrap());
    let profile = EspResetProfile::parse(input.as_deref().unwrap_or(RESET_PROFILE)).unwrap();
    let rom = read_fixture("SIM_ESP_BOOT_ROM");
    let mut board = qemu_board_with_profile(&rom, Some(profile.clone()));
    let app_main = std::env::var("SIM_ESP_BOOT_PC_APP_MAIN").unwrap();
    let app_main = u32::from_str_radix(app_main.trim().trim_start_matches("0x"), 16).unwrap();
    let mut uart = Vec::new();
    let mut main_step = None;
    // Контрольные точки чтения ANA_CONFIG_REG из теста строгой загрузки.
    const BOARD_LAST: u64 = 504_898;
    const QEMU_LAST: usize = 519_764;
    const QEMU_MAIN: usize = 2_403_358;
    assert_eq!(
        qemu_pcs().position(|pc| pc == app_main),
        Some(QEMU_MAIN - 1)
    );
    let mut branch_matched = 0;
    let mut branch = None;
    let mut qemu = qemu_pcs().skip(QEMU_LAST - 1);
    let qemu_uart = read_fixture("SIM_ESP_BOOT_QEMU_UART");
    let marker = b"Hello world!\r\n";
    let drain_uart = |board: &mut dyn BoardSim, uart: &mut Vec<u8>| {
        uart.extend(
            board
                .take_events()
                .into_iter()
                .filter_map(|event| match event.output {
                    BoardOutput::Uart { port: 0, byte } => Some(byte),
                    _ => None,
                }),
        );
    };
    let marker_end = qemu_uart
        .windows(marker.len())
        .position(|line| line == marker)
        .unwrap()
        + marker.len();
    // Полная последовательная передача этого префикса UART0, 8N1 по TRM §26.4.
    // 115200 — CONFIG_ESP_CONSOLE_UART_BAUDRATE образа, 160 МГц — максимум CPU.
    let uart_budget = (marker_end as u64 * 10 * 160_000_000).div_ceil(115_200);
    // Два эталонных потока инструкций QEMU плюс время всей передачи UART.
    for _ in 0..(2 * QEMU_MAIN as u64 + uart_budget) {
        let position = board.esp_boot_position().unwrap();
        if position.step + 1 >= BOARD_LAST && branch.is_none() {
            let pc = qemu
                .next()
                .expect("префикс трассы QEMU после чтения REGI2C");
            if pc == position.pc {
                branch_matched += 1;
            } else {
                // В rtc_clk_cpu_freq_get_config, bеq 0x403d1240:
                // TRM §6.2.4.1 задаёт XTAL/2, QEMU esp32c3_clk.c:100–109 — PLL.
                assert_eq!(branch_matched, 95);
                assert_eq!(position.step + 1, 504_993);
                assert_eq!((position.pc, pc), (0x403d_1244, 0x403d_127c));
                branch = Some((position, pc));
                println!(
                    "После REGI2C совпали {branch_matched} PC, далее BoardSim {:#010x} №{}, QEMU {pc:#010x}",
                    position.pc,
                    position.step + 1
                );
            }
        }
        if let Err(fault) = board.run_cycles(1) {
            eprintln!("{:#?}", board.esp_reset_value_uses());
            drain_uart(&mut *board, &mut uart);
            eprintln!("UART0:\n{}", String::from_utf8_lossy(&uart));
            panic!(
                "шаг {}, PC {:#010x}: {fault}",
                position.step + 1,
                position.pc
            );
        }
        if position.pc == app_main {
            main_step.get_or_insert(position.step + 1);
        }
        // Размер порции сбора событий теста, временем модели не управляет.
        if position.step.is_multiple_of(1024) {
            drain_uart(&mut *board, &mut uart);
            if uart
                .split_inclusive(|byte| *byte == b'\n')
                .any(|line| line == marker)
            {
                break;
            }
        }
    }
    drain_uart(&mut *board, &mut uart);
    assert!(branch.is_some());
    assert_eq!(board.reset_cause(), Some(ResetCause::PowerOn));
    assert!(board.capabilities().blocks_hardware_proof);
    assert!(
        main_step.is_some(),
        "BoardSim не вошёл в app_main: {:?}, сброс {:?}, UART0:\n{}",
        board.esp_boot_position(),
        board.reset_cause(),
        String::from_utf8_lossy(&uart)
    );
    assert!(
        uart.split_inclusive(|byte| *byte == b'\n')
            .any(|line| line == marker),
        "UART0:\n{}",
        String::from_utf8_lossy(&uart),
    );
    assert!(String::from_utf8_lossy(&uart).contains("Calling app_main()"));
    println!(
        "BoardSim, UART0: {}",
        String::from_utf8_lossy(marker).trim_end()
    );
    for used in board.esp_reset_value_uses() {
        assert_eq!(used.entry.kind, EspResetValueKind::Qemu);
        assert_eq!(used.reads, used.read_addresses.values().sum());
        println!(
            "{:#010x} {} {:#010x} ×{} {}",
            used.entry.address, used.entry.field, used.entry.value, used.reads, used.entry.source
        );
        if used.read_addresses.len() > 1 {
            println!("  Слова памяти: {:08x?}", used.read_addresses);
        }
    }
    assert_eq!(board.esp_reset_value_uses().len(), profile.entries().len());
    assert!(
        board
            .esp_virtual_rng_use()
            .is_some_and(|use_| use_.reads > 0)
    );
    println!(
        "BoardSim: app_main {app_main:#010x} #{}",
        main_step.unwrap()
    );
}
