use sim_boards::{BoardFault, EspSystem, SystimerPhase, esp_clock::EspClockSource};

// ESP-IDF v6.1.1 soc/esp32c3/register/soc/{reg_base,system_reg,systimer_reg}.h.
const SYSCLK: u32 = 0x600c_0058;
const CPU_PER: u32 = 0x600c_0008;
const APB_GATE: u32 = 0x600c_0010;
const RESET: u32 = 0x600c_0018;
const GATE_BIT: u32 = 1 << 29;
const TIMER: u32 = 0x6002_3000;
const CONF: u32 = TIMER;
const OP0: u32 = TIMER + 0x04;
const OP1: u32 = TIMER + 0x08;
const LOAD_HI0: u32 = TIMER + 0x0c;
const LOAD_LO0: u32 = TIMER + 0x10;
const LOAD_HI1: u32 = TIMER + 0x14;
const LOAD_LO1: u32 = TIMER + 0x18;
const TARGET_LO0: u32 = TIMER + 0x20;
const TARGET_LO2: u32 = TIMER + 0x30;
const TARGET_CONF1: u32 = TIMER + 0x38;
const TARGET_CONF2: u32 = TIMER + 0x3c;
const VALUE_HI0: u32 = TIMER + 0x40;
const VALUE_LO0: u32 = TIMER + 0x44;
const VALUE_HI1: u32 = TIMER + 0x48;
const VALUE_LO1: u32 = TIMER + 0x4c;
const COMP_LOAD0: u32 = TIMER + 0x50;
const COMP_LOAD1: u32 = TIMER + 0x54;
const COMP_LOAD2: u32 = TIMER + 0x58;
const UNIT_LOAD0: u32 = TIMER + 0x5c;
const UNIT_LOAD1: u32 = TIMER + 0x60;
const ENA: u32 = TIMER + 0x64;
const RAW: u32 = TIMER + 0x68;
const CLR: u32 = TIMER + 0x6c;
const ST: u32 = TIMER + 0x70;
const UNIT0: u32 = 1 << 30;
const UNIT1: u32 = 1 << 29;
const STALL1: u32 = 1 << 26;
const COMP0: u32 = 1 << 24;
const COMP1: u32 = 1 << 23;
const COMP2: u32 = 1 << 22;
const SELECT_UNIT1: u32 = 1 << 31;
const PERIOD_MODE: u32 = 1 << 30;
const UPDATE: u32 = 1 << 30;
const VALID: u32 = 1 << 29;
// PC стенда внутри SRAM1, TRM §3.3.2.
const PC: u32 = 0x4038_0000;

fn read(board: &mut EspSystem, addr: u32) -> u32 {
    board.access(PC, addr, 4, None).unwrap().unwrap()
}

fn write(board: &mut EspSystem, addr: u32, value: u32) {
    assert_eq!(board.access(PC, addr, 4, Some(value)), Ok(Some(0)));
}

fn advance(board: &mut EspSystem, cycles: u64) {
    board.advance_cpu(cycles, board.clocks(), false).unwrap();
}

fn sample0(board: &mut EspSystem) -> u64 {
    write(board, OP0, UPDATE);
    assert_eq!(read(board, OP0), VALID);
    (u64::from(read(board, VALUE_HI0)) << 32) | u64::from(read(board, VALUE_LO0))
}

#[test]
fn частоты_по_таблицам_trm_и_запрет_неизвестных_режимов() {
    let mut board = EspSystem::new(SystimerPhase::FirstAfterThreeXtalCycles);
    // TRM §§6.2.4/16.5 и rtc_clk.c: XTAL/2 после сброса, затем 40/80/160 МГц.
    assert_eq!(read(&mut board, SYSCLK), 1);
    assert_eq!(read(&mut board, CPU_PER), 0x0c);
    assert_eq!(board.wait_gate_cycles(), None);
    assert_eq!(board.clocks().source(), EspClockSource::Xtal);
    assert_eq!(board.clocks().cpu_hz(), (40_000_000, 2));
    write(&mut board, SYSCLK, 2);
    assert_eq!(board.clocks().cpu_hz(), (40_000_000, 3));
    assert_eq!(board.clocks().apb_hz(), (40_000_000, 3));
    for (pll_field, pll) in [(0, 320_000_000), (4, 480_000_000)] {
        for (period_field, cpu) in [(0, 80_000_000), (1, 160_000_000)] {
            write(&mut board, CPU_PER, pll_field | period_field | 0x30);
            write(&mut board, SYSCLK, 1 << 10);
            assert_eq!(board.clocks().source(), EspClockSource::Pll);
            assert_eq!(board.clocks().pll_selected_hz(), pll);
            assert_eq!(board.clocks().cpu_hz(), (cpu, 1));
            assert_eq!(board.clocks().apb_hz(), (80_000_000, 1));
            assert_eq!(board.wait_gate_cycles(), Some(3));
        }
    }
    let rates = board.clocks();
    for (address, bad) in [(SYSCLK, 2 << 10), (SYSCLK, 3 << 10), (CPU_PER, 2)] {
        assert!(matches!(
            board.access(PC, address, 4, Some(bad)),
            Err(BoardFault::Unsupported(_))
        ));
        assert_eq!(board.clocks(), rates);
    }
    assert_eq!(read(&mut board, SYSCLK), 1 << 10);
    assert_eq!(read(&mut board, CPU_PER), 0x35);
}

#[test]
fn время_переключений_не_теряет_четверти_наносекунды() {
    let mut board = EspSystem::new(SystimerPhase::FirstAfterThreeXtalCycles);
    // 50 нс за такт XTAL/2, 6,25 нс после переключения на PLL/3.
    let previous = board.clocks();
    write(&mut board, CPU_PER, 0x0d);
    write(&mut board, SYSCLK, 1 << 10);
    board.advance_cpu(1, previous, false).unwrap();
    advance(&mut board, 1);
    assert_eq!(board.elapsed_quarter_ns(), 225);
    assert_eq!(board.cycles_before(62), 0);
    assert_eq!(board.cycles_before(63), 1);
    advance(&mut board, 1);
    assert_eq!(board.elapsed_quarter_ns(), 250);
    assert_eq!(board.cycles_before(63), 0);

    // Повторяем на выбранном в стенде числе переключений. Две партии по 8 тактов
    // дают 50 нс на PLL/3 и 600 нс на XTAL/3, то есть 650 нс за итерацию.
    let mut branch = board.clone();
    for _ in 0..100_000 {
        advance(&mut board, 8);
        write(&mut board, SYSCLK, 2);
        advance(&mut board, 8);
        write(&mut board, SYSCLK, 1 << 10);
    }
    assert_eq!(board.elapsed_quarter_ns(), 260_000_250);
    // Независимая средняя частота: 65 мс на 16 МГц дают 1 040 000 тиков.
    assert_eq!(sample0(&mut board), 1_040_000);
    advance(&mut branch, 10_400_000);
    assert_eq!(branch.elapsed_quarter_ns(), board.elapsed_quarter_ns());
    assert_eq!(sample0(&mut branch), sample0(&mut board));
    board.reset();
    assert_eq!(board.elapsed_quarter_ns(), 260_000_250);
    assert_eq!(sample0(&mut board), 0);
}

#[test]
fn фаза_xtal_двух_счётчиков_снимок_загрузка_и_stall() {
    use SystimerPhase::*;
    for (phase, first_ns) in [
        (FirstAfterTwoXtalCycles, 50),
        (FirstAfterThreeXtalCycles, 75),
    ] {
        let mut board = EspSystem::new(phase);
        write(&mut board, SYSCLK, 0); // Один такт CPU равен одному такту XTAL, 25 нс.
        write(&mut board, CONF, UNIT0 | UNIT1 | STALL1);
        advance(&mut board, first_ns / 25 - 1);
        assert_eq!(sample0(&mut board), 0);
        advance(&mut board, 1);
        assert_eq!(read(&mut board, VALUE_LO0), 0); // Снимок остаётся прежним.
        assert_eq!(sample0(&mut board), 1);
        write(&mut board, OP0, VALID);
        assert_eq!(read(&mut board, OP0), 0);
        board
            .advance_cpu(5 - first_ns / 25, board.clocks(), true)
            .unwrap();
        assert_eq!(sample0(&mut board), 2);
        write(&mut board, OP1, UPDATE);
        assert_eq!(read(&mut board, VALUE_LO1), 1);
        // TRM §10.4.1: 52-битный счётчик, два тика за 125 нс, переход через ноль.
        write(&mut board, LOAD_HI1, u32::MAX);
        write(&mut board, LOAD_LO1, u32::MAX);
        write(&mut board, UNIT_LOAD1, 0);
        write(&mut board, OP1, UPDATE);
        assert_eq!(read(&mut board, VALUE_LO1), 1);
        write(&mut board, UNIT_LOAD1, 1);
        write(&mut board, OP1, UPDATE);
        assert_eq!(read(&mut board, VALUE_HI1), 0x000f_ffff);
        assert_eq!(read(&mut board, VALUE_LO1), u32::MAX);
        advance(&mut board, 5);
        write(&mut board, OP1, UPDATE);
        assert_eq!(read(&mut board, VALUE_HI1), 0);
        assert_eq!(read(&mut board, VALUE_LO1), 1);
    }
}

#[test]
fn три_компаратора_маска_очистка_и_следующий_фронт() {
    let mut board = EspSystem::new(SystimerPhase::FirstAfterThreeXtalCycles);
    // На стенде COMP0 и COMP2 равны 2 тикам, COMP1 повторяет каждый тик UNIT1.
    write(&mut board, TARGET_LO0, 2);
    write(&mut board, COMP_LOAD0, 1);
    write(&mut board, TARGET_LO0, 9); // Запись без LOAD не влияет на COMP0.
    write(&mut board, TARGET_CONF1, SELECT_UNIT1 | PERIOD_MODE | 1);
    write(&mut board, COMP_LOAD1, 1);
    write(&mut board, TARGET_LO2, 2);
    write(&mut board, TARGET_CONF2, SELECT_UNIT1);
    write(&mut board, COMP_LOAD2, 1);
    write(
        &mut board,
        CONF,
        UNIT0 | UNIT1 | COMP0 | COMP1 | COMP2 | STALL1,
    );
    assert_eq!(board.cycles_until_irq(false), None);
    write(&mut board, ENA, 7);
    assert_eq!(board.cycles_until_irq(false), Some(2));
    assert_eq!(board.cycles_until_irq(true), Some(3));
    advance(&mut board, 1);
    assert_eq!(read(&mut board, RAW), 0);
    advance(&mut board, 1);
    assert_eq!(read(&mut board, RAW), 2);
    assert_eq!(read(&mut board, ST), 2);
    assert_eq!(board.cycles_until_irq(false), Some(0));
    write(&mut board, CLR, 2);
    advance(&mut board, 1);
    assert_eq!(read(&mut board, RAW), 7);
    write(&mut board, ENA, 5);
    assert_eq!(read(&mut board, ST), 5);
    write(&mut board, RAW, 4);
    assert_eq!(read(&mut board, RAW), 3);
    write(&mut board, CLR, 7);
    assert_eq!(board.cycles_until_irq(false), None);
    advance(&mut board, 20);
    assert_eq!(read(&mut board, RAW), 2);
    assert_eq!(board.interrupt_status(), 0);
}

#[test]
fn ранняя_цель_ожидает_оборот_просроченная_ближняя_срабатывает() {
    let mut board = EspSystem::new(SystimerPhase::FirstAfterThreeXtalCycles);
    // По TRM табл. 10.4-2 цель 1 находится далеко позади счётчика 2^52−2.
    write(&mut board, LOAD_HI0, 0x000f_ffff);
    write(&mut board, LOAD_LO0, u32::MAX - 1);
    write(&mut board, UNIT_LOAD0, 1);
    write(&mut board, TARGET_LO0, 1);
    write(&mut board, COMP_LOAD0, 1);
    write(&mut board, CONF, UNIT0 | COMP0);
    write(&mut board, ENA, 1);
    assert_eq!(board.cycles_until_irq(false), Some(4));
    advance(&mut board, 3);
    assert_eq!(sample0(&mut board), 0);
    assert_eq!(read(&mut board, RAW), 0);
    advance(&mut board, 1);
    assert_eq!(sample0(&mut board), 1);
    assert_eq!(board.interrupt_status(), 1);
    write(&mut board, CLR, 1);
    write(&mut board, TARGET_LO0, 0);
    write(&mut board, COMP_LOAD0, 1);
    assert_eq!(board.interrupt_status(), 1);
}

#[test]
fn ошибки_mmio_сброс_и_отдельное_тактирование_apb() {
    let mut board = EspSystem::new(SystimerPhase::FirstAfterThreeXtalCycles);
    assert_eq!(EspSystem::shared_sysreg_mask(APB_GATE), GATE_BIT);
    assert_eq!(EspSystem::shared_sysreg_mask(RESET), GATE_BIT);
    assert_eq!(EspSystem::shared_sysreg_mask(CPU_PER), 0);
    assert_eq!(board.access(PC, TIMER - 4, 4, None), Ok(None));
    assert!(matches!(
        board.access(PC, TIMER + 0x74, 4, None),
        Err(BoardFault::UnsupportedRegister { .. })
    ));
    assert!(matches!(
        board.access(PC, ST, 2, None),
        Err(BoardFault::UnsupportedRegister { .. })
    ));
    assert!(matches!(
        board.access(PC, ST + 2, 4, None),
        Err(BoardFault::Unaligned { .. })
    ));
    assert_eq!(
        board.access(PC, ST, 4, Some(1)),
        Err(BoardFault::WriteToReadOnly { pc: PC, addr: ST })
    );
    assert!(matches!(
        board.access(PC, APB_GATE, 4, Some(GATE_BIT | 1)),
        Err(BoardFault::Unsupported(_))
    ));
    write(&mut board, TARGET_CONF1, PERIOD_MODE | 1);
    assert!(matches!(
        board.access(PC, CONF, 4, Some(UNIT0 | COMP1)),
        Err(BoardFault::Misconfigured(_))
    ));
    assert_eq!(read(&mut board, CONF) & COMP1, 0);

    write(&mut board, COMP_LOAD1, 1);
    write(&mut board, CONF, UNIT0 | COMP1);
    write(&mut board, ENA, 2);
    write(&mut board, APB_GATE, 0);
    assert!(matches!(
        board.access(PC, ST, 4, None),
        Err(BoardFault::Unsupported(_))
    ));
    advance(&mut board, 5);
    assert_eq!(board.interrupt_status(), 2);
    write(&mut board, RESET, GATE_BIT);
    assert_eq!(board.interrupt_status(), 0);
    assert_eq!(board.cycles_until_irq(false), None);
    advance(&mut board, 5);
    write(&mut board, APB_GATE, GATE_BIT);
    assert!(matches!(
        board.access(PC, ST, 4, None),
        Err(BoardFault::Unsupported(_))
    ));
    write(&mut board, RESET, 0);
    assert_eq!(sample0(&mut board), 0);
    assert_eq!(read(&mut board, RAW), 0);
    assert_eq!(board.elapsed_quarter_ns(), 2_000);
}

#[test]
fn слишком_долгий_прогон_не_переполняет_время_и_счётчики() {
    let mut board = EspSystem::new(SystimerPhase::FirstAfterThreeXtalCycles);
    // Граница счётчика CPU и аналитический подсчёт 4 тиков SYSTIMER за 5 тактов XTAL/2.
    let cycles = u64::MAX;
    advance(&mut board, cycles);
    let time = u128::from(cycles) * 200;
    assert_eq!(board.elapsed_quarter_ns(), time);
    assert_eq!(
        sample0(&mut board),
        ((u128::from(cycles) * 4 / 5) % (1 << 52)) as u64
    );
    assert!(matches!(
        board.advance_cpu(1, board.clocks(), false),
        Err(BoardFault::Misconfigured(_))
    ));
    assert_eq!(board.elapsed_quarter_ns(), time);
}

#[test]
fn чтения_mmio_совпадают_с_независимым_прогоном_qemu() {
    // QEMU Espressif 9.2.2, esp_develop_9.2.2_20260417. ELF RV32IMC в SRAM1,
    // -M esp32c3 -icount shift=3,sleep=off -accel tcg,one-insn-per-tb=on
    // -d in_asm -trace enable=memory_region_ops_*. Проба завершилась байтом UART '!'.
    // Ниже 12 ответов из MMIO-трассы, а паузы стенда — два оператора по 2500 раз.
    // COMP0 и COMP1 настроены на 160 тиков (10 мкс по TRM §10.3).
    // 5000 тактов модели дают 31,25 мкс, в QEMU это 40 мкс. Проверяем состояния,
    // снятые сразу после настройки или значительно позже границы первого срабатывания.
    let mut board = EspSystem::new(SystimerPhase::FirstAfterThreeXtalCycles);
    write(&mut board, CPU_PER, 0x0d);
    write(&mut board, SYSCLK, 0x400);
    write(&mut board, CONF, UNIT0);
    write(&mut board, LOAD_HI1, 0x12345);
    write(&mut board, LOAD_LO1, 0x89ab_cdef);
    write(&mut board, UNIT_LOAD1, 1);
    write(&mut board, OP1, UPDATE);
    let mut observed = vec![
        read(&mut board, OP1),
        read(&mut board, VALUE_HI1),
        read(&mut board, VALUE_LO1),
    ];

    write(&mut board, LOAD_HI0, 0);
    write(&mut board, LOAD_LO0, 0);
    write(&mut board, UNIT_LOAD0, 1);
    write(&mut board, TARGET_LO0, 160);
    write(&mut board, COMP_LOAD0, 1);
    write(&mut board, CONF, UNIT0 | COMP0);
    observed.push(read(&mut board, RAW));
    advance(&mut board, 5000);
    observed.extend([read(&mut board, RAW), read(&mut board, ST)]);
    write(&mut board, ENA, 1);
    observed.push(read(&mut board, ST));
    write(&mut board, CLR, 1);
    observed.push(read(&mut board, ST));

    write(&mut board, TARGET_CONF1, PERIOD_MODE | 160);
    write(&mut board, COMP_LOAD1, 1);
    write(&mut board, ENA, 2);
    write(&mut board, CONF, UNIT0 | COMP1);
    observed.push(read(&mut board, ST));
    advance(&mut board, 5000);
    observed.push(read(&mut board, ST));
    write(&mut board, CLR, 2);
    observed.push(read(&mut board, ST));
    advance(&mut board, 5000);
    observed.push(read(&mut board, ST));
    assert_eq!(
        observed,
        [0x2000_0000, 0x12345, 0x89ab_cdef, 0, 1, 0, 1, 0, 0, 2, 0, 2]
    );
}

#[test]
fn записи_штатного_загрузчика_и_приложения_переключают_частоты() {
    // Штатный hello_world ESP-IDF v6.1.1 на QEMU Espressif 9.2.2.
    // Все 16 записей в CPU_PER_CONF и SYSCLK_CONF из memory_region_ops_write
    // до app_main и перезагрузки. Частоты рассчитаны по TRM табл. 6.2-2/6.2-4.
    // QEMU принимает эти транзакции на шине, виртуальные часы QEMU задаёт -icount.
    let transactions = [
        (CPU_PER, 0, (40_000_000, 2), (40_000_000, 2)),
        (SYSCLK, 0x000a_8400, (80_000_000, 1), (80_000_000, 1)),
        (SYSCLK, 0x000a_8401, (80_000_000, 1), (80_000_000, 1)),
        (SYSCLK, 0x000a_8400, (80_000_000, 1), (80_000_000, 1)),
        (SYSCLK, 0x000a_8400, (80_000_000, 1), (80_000_000, 1)),
        (SYSCLK, 0x000a_8001, (40_000_000, 2), (40_000_000, 2)),
        (CPU_PER, 0, (40_000_000, 2), (40_000_000, 2)),
        (SYSCLK, 0x000a_8400, (80_000_000, 1), (80_000_000, 1)),
        (SYSCLK, 0x000a_8401, (80_000_000, 1), (80_000_000, 1)),
        (CPU_PER, 0, (80_000_000, 1), (80_000_000, 1)),
        (CPU_PER, 1, (160_000_000, 1), (80_000_000, 1)),
        (SYSCLK, 0x000a_8400, (160_000_000, 1), (80_000_000, 1)),
        (SYSCLK, 0x000a_8401, (160_000_000, 1), (80_000_000, 1)),
        (SYSCLK, 0x000a_8400, (160_000_000, 1), (80_000_000, 1)),
        (SYSCLK, 0x000a_8400, (160_000_000, 1), (80_000_000, 1)),
        (SYSCLK, 0x000a_8001, (40_000_000, 2), (40_000_000, 2)),
    ];
    let mut board = EspSystem::new(SystimerPhase::FirstAfterThreeXtalCycles);
    for (address, value, cpu, apb) in transactions {
        write(&mut board, address, value);
        assert_eq!(
            (board.clocks().cpu_hz(), board.clocks().apb_hz()),
            (cpu, apb)
        );
    }
}
