use sim_boards::{
    BoardFault,
    esp_dma::{EspDma, EspDmaBeat, EspDmaEof},
    esp_memory::EspMemory,
};

// ESP-IDF v6.1.1 soc/esp32c3/register/soc/gdma_reg.h, карта канала 0.
const BASE: u32 = 0x6003_f000;
const RAW: u32 = BASE;
const ST: u32 = BASE + 0x04;
const ENA: u32 = BASE + 0x08;
const CLR: u32 = BASE + 0x0c;
const IN_CONF0: u32 = BASE + 0x70;
const IN_CONF1: u32 = BASE + 0x74;
const IN_LINK: u32 = BASE + 0x80;
const IN_EOF: u32 = BASE + 0x88;
const IN_ERR_EOF: u32 = BASE + 0x8c;
const IN_PERI: u32 = BASE + 0xa0;
const OUT_CONF0: u32 = BASE + 0xd0;
const OUT_CONF1: u32 = BASE + 0xd4;
const OUT_LINK: u32 = BASE + 0xe0;
const OUT_STATE: u32 = BASE + 0xe4;
const OUT_EOF: u32 = BASE + 0xe8;
const OUT_BEFORE_EOF: u32 = BASE + 0xec;
const OUT_DSCR: u32 = BASE + 0xf0;
const OUT_DSCR_BF0: u32 = BASE + 0xf4;
const OUT_PERI: u32 = BASE + 0x100;
const CHANNEL_STRIDE: u32 = 0xc0;
const MEM_TRANS: u32 = 1 << 4;
const RX_BURST: u32 = 1 << 3;
const AUTO_WRITEBACK: u32 = 1 << 2;
const CHECK_OWNER: u32 = 1 << 12;
const IN_START: u32 = 1 << 22;
const IN_STOP: u32 = 1 << 21;
const IN_AUTO_RETURN: u32 = 1 << 20;
const OUT_START: u32 = 1 << 21;
const OUT_RESTART: u32 = 1 << 22;
const OUT_PARK: u32 = 1 << 23;
const RX_DONE: u32 = 1 << 0;
const RX_EOF: u32 = 1 << 1;
const RX_ERR_EOF: u32 = 1 << 2;
const TX_DONE: u32 = 1 << 3;
const TX_EOF: u32 = 1 << 4;
const RX_DESC_ERR: u32 = 1 << 5;
const TX_DESC_ERR: u32 = 1 << 6;
const RX_DESC_EMPTY: u32 = 1 << 7;
const TX_TOTAL_EOF: u32 = 1 << 8;

// TRM §§2.4.1, 2.4.7. Смещения внутри окна SRAM1 выбраны для этих опытов.
const TX: u32 = 0x3fc8_0400;
const RX: u32 = 0x3fc8_0500;
const SOURCE: u32 = 0x3fc8_0600;
const DEST: u32 = 0x3fc8_0700;
const RAM_END: u32 = 0x3fce_0000;
const OWNER: u32 = 1 << 31;
const EOF: u32 = 1 << 30;
const ERR_EOF: u32 = 1 << 28;
const DESC_BYTES: u32 = 12;
const FIRST_MASK: u32 = 0x000f_ffff; // GDMA_INLINK_ADDR/GDMA_OUTLINK_ADDR.

fn wr(dma: &mut EspDma, addr: u32, value: u32) {
    assert_eq!(dma.access(0, addr, 4, Some(value)).unwrap(), Some(0));
}

fn rd(dma: &mut EspDma, addr: u32) -> u32 {
    dma.access(0, addr, 4, None).unwrap().unwrap()
}

fn desc(memory: &mut EspMemory, addr: u32, flags: u32, buffer: u32, next: u32) {
    for (offset, value) in [(0, flags), (4, buffer), (8, next)] {
        memory.write(0, addr + offset, 4, value).unwrap();
    }
}

fn word(memory: &mut EspMemory, addr: u32) -> u32 {
    memory.read(0, addr, 4).unwrap()
}

fn start(dma: &mut EspDma, rx: u32, tx: u32) {
    wr(dma, OUT_LINK, OUT_START | (tx & FIRST_MASK));
    wr(dma, IN_LINK, IN_START | (rx & FIRST_MASK));
}

fn fixture() -> (EspDma, EspMemory) {
    let mut dma = EspDma::default();
    dma.set_bus_control(true, false);
    let mut memory = EspMemory::default();
    let mut original = vec![0xaa; 8];
    memory.load_ram_segment(DEST, &original).unwrap();
    original[..5].copy_from_slice(&[0x11, 0x22, 0x33, 0x44, 0x55]);
    memory.load_ram_segment(SOURCE, &original).unwrap();
    desc(&mut memory, TX, OWNER | EOF | 0x5005, SOURCE, 0);
    desc(&mut memory, RX, OWNER | 8, DEST, 0);
    wr(&mut dma, IN_CONF0, MEM_TRANS);
    wr(&mut dma, OUT_CONF0, AUTO_WRITEBACK);
    (dma, memory)
}

#[test]
fn copy_and_irq_match_qemu_on_common_bits() {
    // QEMU Espressif 9.2.2, -M esp32c3 -d in_asm, один insn на TB:
    // результат чтения SRAM: 44332211 aaaaaa55 40005005 40005008;
    // INT_RAW/INT_ST = 1b; RX EOF = 3fc80500; после CLR INT_ST = 0.
    // При маршруте ETS_DMA_CH0_INTR_SOURCE на CPU 5 установлен mip.MEIP
    // (0x800), после CLR сброшен. QEMU не заполняет TX EOF addr и TOTAL_EOF
    // в этом опыте: для обоих ниже отдельные ожидания по TRM §§2.4.6, 2.5.
    let (mut dma, mut memory) = fixture();
    start(&mut dma, RX, TX);
    assert_eq!(rd(&mut dma, OUT_LINK) & OUT_PARK, 0);
    assert_eq!(dma.step_memory(&mut memory, 0, 32), Ok(5));
    assert_eq!(word(&mut memory, DEST), 0x4433_2211);
    assert_eq!(word(&mut memory, DEST + 4), 0xaaaa_aa55);
    assert_eq!(word(&mut memory, TX), 0x4000_5005);
    assert_eq!(word(&mut memory, RX), 0x4000_5008);
    assert_eq!(rd(&mut dma, IN_EOF), RX);
    assert_eq!(rd(&mut dma, OUT_EOF), TX);
    assert_eq!(rd(&mut dma, OUT_LINK), OUT_PARK | (TX & FIRST_MASK));

    assert_eq!(rd(&mut dma, RAW) & !TX_TOTAL_EOF, 0x1b);
    assert_eq!(rd(&mut dma, RAW), 0x1b | TX_TOTAL_EOF);
    assert_eq!(rd(&mut dma, ST), 0);
    assert_eq!(
        EspDma::IRQ_STATUS,
        [
            (44, 0x6003_f004, 0x1fff),
            (45, 0x6003_f014, 0x1fff),
            (46, 0x6003_f024, 0x1fff)
        ]
    );
    wr(&mut dma, ENA, RX_EOF | TX_TOTAL_EOF);
    assert_eq!(rd(&mut dma, ST), RX_EOF | TX_TOTAL_EOF);
    wr(&mut dma, CLR, RX_EOF);
    assert_eq!(rd(&mut dma, ST), TX_TOTAL_EOF);
    wr(&mut dma, ENA, 0);
    assert_eq!(rd(&mut dma, ST), 0);
    assert_eq!(rd(&mut dma, RAW), TX_TOTAL_EOF | RX_DONE | TX_DONE | TX_EOF);
    wr(&mut dma, ENA, TX_TOTAL_EOF);
    assert_eq!(rd(&mut dma, ST), TX_TOTAL_EOF);
    wr(&mut dma, CLR, TX_TOTAL_EOF);
    assert_eq!(rd(&mut dma, ST), 0);
}

#[test]
fn chained_copy_splits_rx_and_keeps_channels_independent() {
    let (mut dma, mut memory) = fixture();
    let tx2 = TX + DESC_BYTES;
    let rx2 = RX + DESC_BYTES;
    desc(&mut memory, TX, OWNER | 0x2002, SOURCE, tx2);
    desc(&mut memory, tx2, OWNER | EOF | 0x3003, SOURCE + 2, 0);
    desc(&mut memory, RX, OWNER | 3, DEST, rx2);
    desc(&mut memory, rx2, OWNER | 5, DEST + 3, 0);

    // Канал 2 имеет свои LINK и статус. Запуск только RX ждёт TX той же пары.
    let ch2 = 2 * CHANNEL_STRIDE;
    wr(&mut dma, IN_CONF0 + ch2, MEM_TRANS);
    wr(&mut dma, IN_LINK + ch2, IN_START | (RX & FIRST_MASK));
    assert_eq!(dma.step_memory(&mut memory, 2, 1), Ok(0));
    assert_eq!(word(&mut memory, DEST), 0xaaaa_aaaa);
    assert_eq!(dma.step_memory(&mut memory, 0, 1), Ok(0));
    wr(&mut dma, OUT_LINK + ch2, OUT_START | (TX & FIRST_MASK));
    assert_eq!(dma.step_memory(&mut memory, 2, 2), Ok(2));
    assert_eq!(rd(&mut dma, BASE + 0x20), TX_DONE);
    assert_eq!(rd(&mut dma, OUT_DSCR + ch2), tx2);
    assert_eq!(rd(&mut dma, OUT_DSCR_BF0 + ch2), TX);
    assert_eq!(rd(&mut dma, OUT_STATE + ch2), tx2 & 0x3_ffff); // OUTLINK_DSCR_ADDR[17:0].
    assert_eq!(word(&mut memory, RX), OWNER | 3);
    assert_eq!(dma.step_memory(&mut memory, 2, 16), Ok(3));
    assert_eq!(word(&mut memory, DEST), 0x4433_2211);
    assert_eq!(word(&mut memory, DEST + 4), 0xaaaa_aa55);
    assert_eq!(word(&mut memory, RX), 0x3003);
    assert_eq!(word(&mut memory, rx2), EOF | 0x2005);
    assert_eq!(word(&mut memory, TX), OWNER | 0x2002);
    assert_eq!(word(&mut memory, tx2), OWNER | EOF | 0x3003);
    assert_eq!(rd(&mut dma, IN_EOF + ch2), rx2);
    assert_eq!(rd(&mut dma, OUT_EOF + ch2), tx2);
    assert_eq!(rd(&mut dma, OUT_BEFORE_EOF + ch2), TX);
    assert_eq!(rd(&mut dma, RAW), 0);
    assert_eq!(rd(&mut dma, BASE + 0x20), 0x11b);
}

#[test]
fn owner_errors_latch_irq_and_require_channel_reset() {
    // QEMU в том же опыте: повтор с CPU owner и IN_CHECK_OWNER дал RAW=0x20.
    let (mut dma, mut memory) = fixture();
    wr(&mut dma, IN_CONF1, CHECK_OWNER);
    wr(&mut dma, ENA, RX_DESC_ERR);
    desc(&mut memory, RX, 8, DEST, 0);
    start(&mut dma, RX, TX);
    let error = dma.step_memory(&mut memory, 0, 10).unwrap_err();
    assert!(error.to_string().contains("0x3fc80500"));
    assert!(error.to_string().contains("принадлежит CPU"));
    assert_eq!(rd(&mut dma, RAW), 0x20);
    assert_eq!(rd(&mut dma, ST), RX_DESC_ERR);
    assert_eq!(word(&mut memory, DEST), 0xaaaa_aaaa);
    assert!(
        dma.access(0, IN_LINK, 4, Some(IN_START | (RX & FIRST_MASK)))
            .is_err()
    );
    wr(&mut dma, IN_CONF0, MEM_TRANS | 1); // GDMA_IN_RST, бит 0.
    wr(&mut dma, IN_CONF0, MEM_TRANS);
    wr(&mut dma, CLR, RX_DESC_ERR);
    desc(&mut memory, RX, OWNER | 8, DEST, 0);
    wr(&mut dma, IN_LINK, IN_START | (RX & FIRST_MASK));
    assert_eq!(dma.step_memory(&mut memory, 0, 10), Ok(5));
    assert_eq!(rd(&mut dma, ST), 0);

    let (mut dma, mut memory) = fixture();
    wr(&mut dma, OUT_CONF1, CHECK_OWNER);
    desc(&mut memory, TX, EOF | 0x5005, SOURCE, 0);
    start(&mut dma, RX, TX);
    assert!(
        dma.step_memory(&mut memory, 0, 10)
            .unwrap_err()
            .to_string()
            .contains("0x3fc80400")
    );
    assert_eq!(rd(&mut dma, RAW), TX_DESC_ERR);
}

#[test]
fn invalid_descriptors_and_buffers_report_the_descriptor_address() {
    let (mut dma, mut memory) = fixture();
    for address in [TX + 1, RAM_END - 8, RAM_END, 0x3fc0_0000, TX + 0x20] {
        let mut dma = dma.clone();
        start(&mut dma, RX, address);
        let error = dma.step_memory(&mut memory, 0, 10).unwrap_err();
        assert!(error.to_string().contains(&format!("{address:#010x}")));
        assert_eq!(rd(&mut dma, RAW), TX_DESC_ERR);
    }
    for address in [
        RAM_END - 4, // Начало допустимо, последние байты выходят из SRAM1.
        0x4038_0000, // Алиас IBUS, TRM §3.3.2.
        0x5000_0000, // RTC FAST, TRM §3.3.2.
        0x3c00_0000, // Flash DBUS, TRM §3.3.3.
        0x6000_0000, // MMIO, TRM §3.3.5.
        u32::MAX,
    ] {
        let mut dma = dma.clone();
        desc(&mut memory, RX, OWNER | 8, address, 0);
        start(&mut dma, RX, TX);
        let error = dma.step_memory(&mut memory, 0, 10).unwrap_err();
        assert!(error.to_string().contains("0x3fc80500"));
        assert!(error.to_string().contains(&format!("{address:#010x}")));
        assert_eq!(rd(&mut dma, RAW), RX_DESC_ERR);
        assert_eq!(word(&mut memory, DEST), 0xaaaa_aaaa);
    }

    desc(&mut memory, RX, OWNER | 8, DEST, 0);
    let mut bad_link = dma.clone();
    desc(&mut memory, TX, OWNER | EOF | 0x1001, SOURCE, 0x5000_0000);
    start(&mut bad_link, RX, TX);
    let error = bad_link
        .step_memory(&mut memory, 0, 1)
        .unwrap_err()
        .to_string();
    assert!(error.contains("0x3fc80400") && error.contains("0x50000000"));
    assert_eq!(rd(&mut bad_link, RAW), TX_DESC_ERR);
    assert_eq!(word(&mut memory, DEST), 0xaaaa_aaaa);

    // Адрес следующего дескриптора допустим, но его DW0 не инициализирован.
    let next = TX + 0x20;
    desc(&mut memory, TX, OWNER | 0x1001, SOURCE, next);
    start(&mut dma, RX, TX);
    assert_eq!(dma.step_memory(&mut memory, 0, 1), Ok(1));
    assert!(
        dma.step_memory(&mut memory, 0, 1)
            .unwrap_err()
            .to_string()
            .contains(&format!("{next:#010x}"))
    );
    assert_eq!(word(&mut memory, DEST), 0xaaaa_aa11);
    assert_eq!(rd(&mut dma, RAW), TX_DONE | TX_DESC_ERR);
}

#[test]
fn accepts_the_inclusive_last_byte_and_the_first_descriptor_in_sram() {
    let (mut dma, mut memory) = fixture();
    let rx = 0x3fc8_0000; // TRM §2.4.7: первый адрес SRAM1 DBUS.
    let tx = RAM_END - DESC_BYTES;
    let last_byte = tx - 1;
    memory.write(0, last_byte, 1, 0x65).unwrap();
    desc(&mut memory, tx, OWNER | EOF | 0x1001, last_byte, 0);
    desc(&mut memory, rx, OWNER | 1, rx + DESC_BYTES, 0);
    start(&mut dma, rx, tx);
    assert_eq!(dma.step_memory(&mut memory, 0, 2), Ok(1));
    assert_eq!(memory.read(0, rx + DESC_BYTES, 1).unwrap(), 0x65);
    assert_eq!(rd(&mut dma, OUT_EOF), tx);

    let (mut dma, mut memory) = fixture();
    desc(&mut memory, RX, OWNER | 1, RAM_END - 1, 0);
    desc(&mut memory, TX, OWNER | EOF | 0x1001, SOURCE, 0);
    start(&mut dma, RX, TX);
    assert_eq!(dma.step_memory(&mut memory, 0, 2), Ok(1));
    assert_eq!(memory.read(0, RAM_END - 1, 1).unwrap(), 0x11);
}

#[test]
fn validates_length_burst_alignment_and_rx_space() {
    for (flags, conf, buffer) in [
        (OWNER, MEM_TRANS, DEST),
        (OWNER | 8, MEM_TRANS | RX_BURST, DEST + 1),
        (OWNER | 7, MEM_TRANS | RX_BURST, DEST),
    ] {
        let (mut dma, mut memory) = fixture();
        desc(&mut memory, RX, flags, buffer, 0);
        wr(&mut dma, IN_CONF0, conf);
        start(&mut dma, RX, TX);
        assert!(dma.step_memory(&mut memory, 0, 8).is_err());
        assert_eq!(rd(&mut dma, RAW), RX_DESC_ERR);
        assert_eq!(word(&mut memory, DEST), 0xaaaa_aaaa);
    }
    let (mut dma, mut memory) = fixture();
    desc(&mut memory, TX, OWNER | 0x6005, SOURCE, 0);
    start(&mut dma, RX, TX);
    assert!(dma.step_memory(&mut memory, 0, 8).is_err());
    assert_eq!(rd(&mut dma, RAW), TX_DESC_ERR);

    let (mut dma, mut memory) = fixture();
    desc(&mut memory, RX, OWNER | 3, DEST, 0);
    start(&mut dma, RX, TX);
    let error = dma.step_memory(&mut memory, 0, 8).unwrap_err();
    assert!(error.to_string().contains("0x3fc80500"));
    assert_eq!(rd(&mut dma, RAW), RX_DONE | RX_DESC_EMPTY);
    assert_eq!(word(&mut memory, RX), 0x3003);
    assert_eq!(word(&mut memory, DEST), 0xaa33_2211);
}

#[test]
fn reset_and_stop_cancel_partial_work_and_snapshot_can_continue() {
    let (mut dma, mut memory) = fixture();
    start(&mut dma, RX, TX);
    assert_eq!(dma.step_memory(&mut memory, 0, 2), Ok(2));
    let mut copied_dma = dma.clone();
    let mut copied_memory = memory.clone();
    let mut stopped_dma = dma.clone();
    let mut stopped_memory = memory.clone();
    wr(&mut stopped_dma, IN_LINK, IN_STOP | (RX & FIRST_MASK));
    assert_eq!(stopped_dma.step_memory(&mut stopped_memory, 0, 8), Ok(0));
    assert_eq!(word(&mut stopped_memory, DEST), 0xaaaa_2211);

    dma.set_bus_control(false, false);
    assert_eq!(dma.step_memory(&mut memory, 0, 8), Ok(0));
    dma.set_bus_control(true, false);
    assert_eq!(dma.step_memory(&mut memory, 0, 1), Ok(1));
    assert_eq!(word(&mut memory, DEST), 0xaa33_2211);
    assert_eq!(word(&mut memory, RX), OWNER | 8);
    assert_eq!(word(&mut memory, TX), OWNER | EOF | 0x5005);
    dma.set_bus_control(true, true);
    assert_eq!(dma.step_memory(&mut memory, 0, 8), Ok(0));
    dma.set_bus_control(true, false);
    assert_eq!(dma.step_memory(&mut memory, 0, 8), Ok(0));
    assert_eq!(word(&mut memory, DEST), 0xaa33_2211);
    assert_eq!(rd(&mut dma, RAW), 0);
    assert_eq!(rd(&mut dma, OUT_LINK), OUT_PARK);
    assert_eq!(copied_dma.step_memory(&mut copied_memory, 0, 8), Ok(3));
    assert_eq!(word(&mut copied_memory, DEST), 0x4433_2211);
    assert_ne!(word(&mut memory, DEST), word(&mut copied_memory, DEST));
    copied_dma.reset();
    assert_eq!(rd(&mut copied_dma, RAW), 0);
    assert_eq!(rd(&mut copied_dma, IN_EOF), 0);
    assert!(copied_dma.access(0, ENA, 4, Some(RX_EOF)).is_err());
}

#[test]
fn peripheral_requests_and_restart_follow_a_new_tail() {
    // TRM табл. 2.4-1: запрос 0 направлен к SPI2. Здесь подаются байты шины.
    let (mut dma, mut memory) = fixture();
    wr(&mut dma, IN_CONF0, 0);
    wr(&mut dma, IN_PERI, 0);
    wr(&mut dma, OUT_PERI, 0);
    wr(&mut dma, OUT_CONF0, 0);
    desc(&mut memory, TX, OWNER | EOF | 0x1001, SOURCE, 0);
    start(&mut dma, RX, TX);
    assert_eq!(dma.step_memory(&mut memory, 0, 10), Ok(0));
    let beat = dma.transmit(&mut memory, 0).unwrap().unwrap();
    assert_eq!(
        beat,
        EspDmaBeat {
            byte: 0x11,
            eof: EspDmaEof::Success
        }
    );
    assert!(dma.receive(&mut memory, 0, beat).unwrap());
    assert_eq!(word(&mut memory, DEST), 0xaaaa_aa11);
    assert_eq!(dma.transmit(&mut memory, 0), Ok(None));
    assert_eq!(word(&mut memory, TX), OWNER | EOF | 0x1001);

    let appended = TX + DESC_BYTES;
    desc(&mut memory, appended, OWNER | EOF | 0x1001, SOURCE + 1, 0);
    memory.write(0, TX + 8, 4, appended).unwrap();
    wr(&mut dma, OUT_LINK, OUT_RESTART | (TX & FIRST_MASK));
    assert_eq!(
        dma.transmit(&mut memory, 0),
        Ok(Some(EspDmaBeat {
            byte: 0x22,
            eof: EspDmaEof::Success
        }))
    );
    assert_eq!(rd(&mut dma, OUT_EOF), appended);
    assert_eq!(rd(&mut dma, OUT_BEFORE_EOF), TX);
    assert!(dma.receive(&mut memory, 0, beat).is_err());
    assert_ne!(rd(&mut dma, RAW) & RX_DESC_EMPTY, 0);
    wr(&mut dma, OUT_PERI + CHANNEL_STRIDE, 0);
    assert!(dma.transmit(&mut memory, 0).is_err());
    assert!(dma.transmit(&mut memory, 1).is_err()); // Значение 1 зарезервировано.
}

#[test]
fn restart_active_tx_uses_the_updated_pointer_without_a_false_total_eof() {
    let (mut dma, mut memory) = fixture();
    wr(&mut dma, IN_CONF0, 0);
    wr(&mut dma, OUT_PERI, 0);
    let appended = TX + DESC_BYTES;
    desc(&mut memory, TX, OWNER | 0x2002, SOURCE, 0);
    desc(&mut memory, appended, OWNER | EOF | 0x1001, SOURCE + 2, 0);
    wr(&mut dma, OUT_LINK, OUT_START | (TX & FIRST_MASK));
    let first = dma.transmit(&mut memory, 0).unwrap().unwrap();
    assert_eq!(
        first,
        EspDmaBeat {
            byte: 0x11,
            eof: EspDmaEof::None
        }
    );
    memory.write(0, TX + 8, 4, appended).unwrap();
    wr(&mut dma, OUT_LINK, OUT_RESTART | (TX & FIRST_MASK));
    let second = dma.transmit(&mut memory, 0).unwrap().unwrap();
    assert_eq!(
        second,
        EspDmaBeat {
            byte: 0x22,
            eof: EspDmaEof::None
        }
    );
    assert_eq!(rd(&mut dma, RAW), TX_DONE);
    let third = dma.transmit(&mut memory, 0).unwrap().unwrap();
    assert_eq!(
        third,
        EspDmaBeat {
            byte: 0x33,
            eof: EspDmaEof::Success
        }
    );
    assert_eq!(rd(&mut dma, RAW), TX_DONE | TX_EOF | TX_TOTAL_EOF);
}

#[test]
fn rx_error_eof_never_sets_success_and_requires_uhcio() {
    let (mut dma, mut memory) = fixture();
    let beat = EspDmaBeat {
        byte: 0x62,
        eof: EspDmaEof::Error,
    };
    assert!(dma.receive(&mut memory, 0, beat).is_err());
    // TRM табл. 2.4-1 и §2.5: PERI_SEL=2 для UHCIO, единственного источника ERR_EOF.
    wr(&mut dma, IN_CONF0, 0);
    wr(&mut dma, IN_PERI, 2);
    wr(
        &mut dma,
        IN_LINK,
        IN_START | IN_AUTO_RETURN | (RX & FIRST_MASK),
    );
    assert!(matches!(
        dma.receive(&mut memory, 2, beat),
        Err(BoardFault::Unsupported(_))
    ));
    assert_eq!(word(&mut memory, DEST), 0xaaaa_aaaa);
    wr(&mut dma, IN_LINK, RX & FIRST_MASK);
    assert_eq!(dma.receive(&mut memory, 2, beat), Ok(true));
    assert_eq!(word(&mut memory, DEST), 0xaaaa_aa62);
    assert_eq!(word(&mut memory, RX), ERR_EOF | 0x1008);
    assert_eq!(rd(&mut dma, IN_ERR_EOF), RX);
    assert_eq!(rd(&mut dma, IN_EOF), 0);
    assert_eq!(rd(&mut dma, RAW), RX_DONE | RX_ERR_EOF);
}

#[test]
fn unsupported_mmio_remains_explicit() {
    let (mut dma, _) = fixture();
    // FIFO_STATUS по gdma_reg.h, отдельная модель FIFO здесь отсутствует.
    assert!(matches!(
        dma.access(0, BASE + 0x78, 4, None),
        Err(BoardFault::UnsupportedRegister { .. })
    ));
    assert!(matches!(
        dma.access(0, ST, 4, Some(0)),
        Err(BoardFault::WriteToReadOnly { .. })
    ));
    assert!(matches!(
        dma.access(0, RAW, 4, Some(RX_EOF)),
        Err(BoardFault::WriteToReadOnly { .. })
    ));
    assert!(matches!(
        dma.access(0, ST + 1, 4, None),
        Err(BoardFault::Unaligned { .. })
    ));
    assert!(matches!(
        dma.access(0, ST, 2, None),
        Err(BoardFault::UnsupportedRegister { .. })
    ));
    assert_eq!(dma.access(0, BASE - 4, 4, None), Ok(None));
}
