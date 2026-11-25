//! Сквозные проверки raw flash, начиная с инструкции в векторе сброса.

use sim_boards::{
    BoardFault, BoardId, BoardSim, EspBootOptions, EspBootPosition, SystimerPhase, esp_image,
    new_board,
};

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
const SPI1: u32 = 0x6000_2000;

const EBREAK: u32 = 0x0010_0073; // RISC-V Unprivileged ISA, EBREAK.

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

fn raw_image() -> Vec<u8> {
    // esp_app_format.h: 1 сегмент, DIO, 2 МиБ и 80 МГц, CHIP_ID ESP32-C3.
    let mut bin = vec![0xe9, 1, 2, 0x1f];
    bin.extend(SRAM.to_le_bytes());
    bin.extend([0xee, 0, 0, 0, 5, 0, 0, 0, 0, 199, 0, 0, 0, 0, 0, 0]);
    bin.extend(SRAM.to_le_bytes());
    bin.extend((size_of::<u32>() as u32).to_le_bytes());
    bin.extend(EBREAK.to_le_bytes());
    // esp_image_format.c: XOR payload и выравнивание контрольной суммы.
    bin.resize(bin.len() / 16 * 16 + 15, 0);
    bin.push(EBREAK.to_le_bytes().into_iter().fold(0xef, |a, b| a ^ b));
    bin
}

fn board_for(probe: &Probe, phase: Option<SystimerPhase>) -> Box<dyn BoardSim> {
    const TABLE: u32 = esp_image::DEFAULT_PARTITION_TABLE_OFFSET;
    // partitions_singleapp.csv и gen_esp32part.py: заводской app с 0x10000.
    const APP: u32 = 0x10000;
    let mut flash = raw_image();
    flash.resize(TABLE as usize, 0xff);
    flash.extend([0xaa, 0x50, 0, 0]);
    flash.extend(APP.to_le_bytes());
    flash.extend(APP.to_le_bytes());
    flash.extend(b"factory");
    flash.resize(TABLE as usize + 32, 0);
    flash.resize(APP as usize, 0xff);
    flash.extend(raw_image());
    let mut board = new_board(BoardId::Esp32C3);
    board
        .load_esp_flash_with_options(
            &flash,
            TABLE,
            &probe.image(),
            &[0; EFUSE_WORDS],
            [true; 3],
            EspBootOptions {
                systimer_phase: phase,
                flash_readback: None,
            },
        )
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
        let header = u32::from_le_bytes(raw_image()[..size_of::<u32>()].try_into().unwrap());
        probe.expect_read(SPI1 + 0x58, header);
        let mut enable_pc = None;
        if enable_cache {
            probe.put(CACHE_ENABLE, 1); // EXTMEM_ICACHE_ENABLE.
            enable_pc = Some(probe.pc() - size_of::<u32>() as u32);
        }
        let success_pc = probe.pc();
        probe.0.push(EBREAK);
        let mut board = board_for(&probe, None);
        let stop = match enable_pc {
            None => BoardFault::Breakpoint { pc: success_pc },
            Some(pc) => BoardFault::UnsupportedRegister {
                pc,
                addr: CACHE_ENABLE,
                size: size_of::<u32>() as u8,
                write: true,
                block: "EXTMEM: окна IROM/DROM не подключены к шине CPU",
            },
        };
        assert_eq!(board.run_cycles(probe.0.len() as u64), Err(stop.clone()));
        assert_eq!(
            board.esp_boot_position().unwrap().pc,
            enable_pc.unwrap_or(success_pc)
        );
        assert_eq!(board.run_cycles(1), Err(stop));
    }
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

/// Трасса QEMU 9.2.2 из `-accel tcg,one-insn-per-tb=on -d in_asm,exec,nochain`.
/// Остальные параметры, образ и UART описаны в firmware-tests/esp_sdk/SOURCES.md.
/// SIM_ESP_BOOT_READBACK — результат `pmemsave 0x6000882c 336 readback.bin` при -S.
#[test]
#[ignore = "нужны штатный образ SDK, дамп ROM, eFuse, UART и трасса QEMU"]
fn штатный_образ_совпадает_с_qemu_до_первого_расхождения() {
    let read = |key| std::fs::read(std::env::var(key).expect(key)).expect(key);
    let flash = read("SIM_ESP_BOOT_FLASH");
    let rom = read("SIM_ESP_BOOT_ROM");
    let fuse = read("SIM_ESP_BOOT_READBACK");
    assert_eq!(fuse.len(), EFUSE_WORDS * size_of::<u32>());
    let fuse: Vec<u32> = fuse
        .chunks_exact(size_of::<u32>())
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
    // QEMU x/1wx 0x60004038 (TRM §5.8 GPIO_STRAP_REG) возвращает 0x08.
    board
        .load_esp_flash(
            &flash,
            esp_image::DEFAULT_PARTITION_TABLE_OFFSET,
            &rom,
            &fuse,
            [false, false, true],
        )
        .unwrap();
    let trace = String::from_utf8(read("SIM_ESP_BOOT_QEMU_PC")).unwrap();
    let pcs = trace
        .lines()
        .filter(|line| line.starts_with("Trace "))
        .map(|line| {
            let fields = line.split_once('[').unwrap().1;
            u32::from_str_radix(fields.split('/').nth(1).unwrap(), 16).unwrap()
        });

    // Из потактовой трассы. Первый разный PC следует за bnez в 0x40049870.
    // QEMU x/1wx 0x600c0058 даёт 0xa8401 (PLL), TRM §6.2.4.1 задаёт XTAL/2.
    const MATCHED: u64 = 5701;
    let mut pcs = pcs.enumerate();
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

    // Первая неподдержанная запись на ветке XTAL/2. UART0 RX: io_mux_reg.h,
    // PERIPHS_IO_MUX_U0RXD_U, FUNC_U0RXD_U0RXD (MCU_SEL=0).
    const STOP: EspBootPosition = EspBootPosition {
        step: 5771,
        pc: 0x4004_c29c,
    };
    assert_eq!(
        board.run_cycles(STOP.step - MATCHED).unwrap().cycle,
        STOP.step
    );
    assert_eq!(board.esp_boot_position(), Some(STOP));
    let fault = BoardFault::UnsupportedRegister {
        pc: STOP.pc,
        addr: 0x6000_9054,
        write: true,
        size: size_of::<u32>() as u8,
        block: "IO MUX вне GPIO Matrix",
    };
    assert_eq!(board.run_cycles(1), Err(fault.clone()));
    assert_eq!(board.run_cycles(1), Err(fault.clone()));
    assert_eq!(board.esp_boot_position(), Some(STOP));
    assert_eq!(board.cycles(), STOP.step);
    assert!(board.uart_take().is_empty());
    println!(
        "Совпали {MATCHED} PC. Отказ модели на шаге {}: {fault}",
        STOP.step + 1
    );
}
