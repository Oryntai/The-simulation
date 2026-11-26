//! Граница между платой и планировщиком. Время задаётся целыми тактами платы.

use crate::{BoardFault, BoardId};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PwmOutput {
    pub duty: f64,
    pub period_s: f64,
    pub high_s: f64,
}

impl PwmOutput {
    pub(crate) fn new(duty: f64, period_s: f64) -> Self {
        Self {
            duty,
            period_s,
            high_s: period_s * duty,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OutputPort {
    Gpio(u32),
    Pwm(u32),
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PortValue {
    Logic(bool),
    Pwm(PwmOutput),
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PortSample {
    pub cycle: u64,
    pub port: OutputPort,
    /// Без активного вывода образца нет. Нулевой ШИМ активного канала остаётся образцом.
    pub value: Option<PortValue>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ResetCause {
    PowerOn,
    Brownout,
    Watchdog,
    Software,
    Host,
    /// Холодный старт цифрового домена после Deep-sleep ESP32-C3.
    DeepSleep,
}

/// Режим питания ESP32-C3 в пределах модели без работающего RF.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EspSleepMode {
    Modem,
    Light,
    Deep,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EspWake {
    pub mode: EspSleepMode,
    /// Граница обнаружения сигнала на шкале платы.
    pub cycle: u64,
    pub rtc_slow_tick: u64,
    pub timer: bool,
    /// Номера выводов в битовой маске. При одновременном будильнике верны обе причины.
    pub gpio_mask: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EspSleepStatus {
    pub mode: EspSleepMode,
    /// Такт завершения инструкции SLEEP_EN, пока плата спит.
    pub entered_at: Option<u64>,
    /// Аппаратный 48-битный счётчик на номинальной частоте RC_SLOW.
    pub rtc_slow_ticks: u64,
    pub last_wake: Option<EspWake>,
}

/// Состояние буферов и подтяжки физического контакта в рабочей периферии.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GpioMode {
    Output(bool),
    Input,
    PullUp,
    PullDown,
    HighImpedance,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SpiMode {
    Mode0,
    Mode1,
    Mode2,
    Mode3,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BitOrder {
    MsbFirst,
    LsbFirst,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TransferStatus {
    Complete,
    /// В смоделированном профиле I²C — NACK первого адреса.
    Nack,
    /// Прервано снятием питания или сбросом.
    Cancelled,
    /// Ответ SPI отсутствует или целевое устройство отвергло настройки.
    NoResponse,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct I2cRequest {
    pub bus: u32,
    pub id: u64,
    pub address: u8,
    pub write: Vec<u8>,
    pub read_len: usize,
    pub repeated_start: bool,
    pub cycles_per_bit: u64,
    pub address_ack_cycle: u64,
    pub complete_cycle: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SpiRequest {
    pub bus: u32,
    pub id: u64,
    pub chip_select: u8,
    pub mode: SpiMode,
    pub write_order: BitOrder,
    pub read_order: BitOrder,
    /// Байты в том порядке, в котором контроллер выставляет их на линии.
    /// Порядок битов внутри каждого байта задают отдельные поля.
    pub write: Vec<u8>,
    pub cycles_per_bit: u64,
    pub complete_cycle: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Packet {
    pub bytes: Vec<u8>,
    /// RSSI задаётся радиоканалом на входе. На выходе МК его ещё нет.
    pub rssi_dbm: Option<i8>,
}

/// Номера логических портов ABI пакетного адаптера C3, определённые в esp_radio.rs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u32)]
pub enum EspRadioPort {
    // Порт 0 остаётся за прежним ABI виртуального МК.
    Wifi = 1,
    Ble = 2,
    EspNow = 3,
}

impl EspRadioPort {
    pub const fn max_bytes(self) -> usize {
        match self {
            // ESP-IDF: esp_wifi.h, esp_wifi_80211_tx (верхняя граница буфера).
            Self::Wifi => 1500,
            // ESP-IDF: NimBLE ble_att.h, BLE_ATT_ATTR_MAX_LEN. MTU задаёт маршрут.
            Self::Ble => 512,
            // ESP-IDF: esp_now.h, ESP_NOW_MAX_DATA_LEN_V2.
            Self::EspNow => 1470,
        }
    }
}

/// Результат пакетной модели, который видит адаптер прошивки.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RadioStatus {
    Pending,
    MacConfirmed,
    BroadcastSent,
    ApplicationConfirmed,
    ApplicationTimedOut,
    Failed,
    NotAnalyzed(&'static str),
}

/// ABI пакетного адаптера C3. Блок команды состоит из little-endian слов
/// (функция, порт, размер, статус, RSSI), затем байты пакета. При SEND
/// длину задаёт гость, при RECEIVE гость задаёт ёмкость и получает длину.
/// Коды ABI определены здесь и не соответствуют кодам возврата ESP-IDF.
pub struct EspRadioAbi;

impl EspRadioAbi {
    // RISC-V Unprivileged ISA, таблица major opcodes: custom-0, биты 6:0 = 0001011.
    pub const INSTRUCTION: u32 = 0x0000_000b;
    pub const INSTRUCTION_BYTES: u32 = size_of::<u32>() as u32;
    pub const CALL_BYTES: u32 = Self::INSTRUCTION_BYTES + size_of::<u32>() as u32;
    pub const FUNCTION: usize = 0;
    pub const PORT: usize = Self::FUNCTION + size_of::<u32>();
    pub const LENGTH: usize = Self::PORT + size_of::<u32>();
    pub const STATUS: usize = Self::LENGTH + size_of::<u32>();
    pub const RSSI: usize = Self::STATUS + size_of::<u32>();
    pub const PAYLOAD_OFFSET: usize = Self::RSSI + size_of::<u32>();

    pub const OPEN: u32 = 0;
    pub const SEND: u32 = 1;
    pub const RECEIVE: u32 = 2;
    pub const TX_STATUS: u32 = 3;
    /// Отправить ответ приложения для последнего прочитанного ESP-NOW пакета.
    pub const ACK: u32 = 4;

    pub const OK: u32 = 0;
    pub const EMPTY: u32 = 1;
    pub const BUSY: u32 = 2;
    pub const BUFFER_TOO_SMALL: u32 = 3;
    pub const PENDING: u32 = 4;
    pub const MAC_CONFIRMED: u32 = 5;
    pub const BROADCAST_SENT: u32 = 6;
    pub const APPLICATION_CONFIRMED: u32 = 7;
    pub const APPLICATION_TIMED_OUT: u32 = 8;
    pub const FAILED: u32 = 9;
    pub const NOT_ANALYZED: u32 = 10;
    /// Пустой RSSI вне представимого диапазона i8 (дБм).
    pub const RSSI_UNAVAILABLE: i32 = i32::MIN;
}

#[derive(Clone, Debug, PartialEq)]
pub enum BoardInput {
    Gpio {
        pin: u32,
        high: bool,
    },
    Adc {
        channel: u32,
        volts: f64,
    },
    /// Вход без определённого напряжения, если схема обрыва не задаёт уровень.
    DisconnectAdc {
        channel: u32,
    },
    /// Байты уже доставлены на вход линии. Для аппаратных UART каждый кадр
    /// в очереди занимает 10 битовых интервалов до попадания в FIFO.
    Uart {
        port: u32,
        bytes: Vec<u8>,
    },
    I2cReply {
        bus: u32,
        id: u64,
        status: TransferStatus,
        bytes: Vec<u8>,
    },
    SpiReply {
        bus: u32,
        id: u64,
        status: TransferStatus,
        bytes: Vec<u8>,
    },
    Radio {
        port: u32,
        packet: Packet,
    },
    RadioResult {
        port: u32,
        sent_cycle: u64,
        status: RadioStatus,
    },
    /// Адрес ответа приложения для только что принятого BoardInput::Radio.
    RadioReplyId {
        port: u32,
        id: u64,
    },
    Power {
        on: bool,
    },
    /// Срабатывание детектора питания. На отпускании вновь стартует загруженный образ.
    Brownout {
        asserted: bool,
    },
    /// При остановленном ядре аппаратное время и watchdog продолжаются.
    Execution {
        enabled: bool,
    },
}

#[derive(Clone, Debug, PartialEq)]
pub struct InputEvent {
    pub cycle: u64,
    pub input: BoardInput,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InputOutcome {
    Applied,
    Unchanged,
    Uart { accepted: usize, dropped: usize },
    PacketDropped,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum BoardOutput {
    Uart {
        port: u32,
        byte: u8,
    },
    Radio {
        port: u32,
        packet: Packet,
    },
    RadioStatus {
        port: u32,
        sent_cycle: u64,
        status: RadioStatus,
    },
    RadioAck {
        port: u32,
        id: u64,
    },
    I2c(I2cRequest),
    Spi(SpiRequest),
    I2cDone {
        bus: u32,
        id: u64,
        address: u8,
        status: TransferStatus,
        bytes: Vec<u8>,
    },
    SpiDone {
        bus: u32,
        id: u64,
        chip_select: u8,
        status: TransferStatus,
        bytes: Vec<u8>,
    },
    Reset(ResetCause),
    Fault(BoardFault),
    Power {
        on: bool,
    },
    Execution {
        enabled: bool,
    },
    Halt {
        code: u32,
    },
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct OutputEvent {
    pub cycle: u64,
    pub output: BoardOutput,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Advance {
    pub cycle: u64,
    /// Запрос шины или пакетного адаптера следует забрать через `take_events`.
    pub bus_request_pending: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Capabilities {
    pub cpu_hz: u64,
    pub adc_channels: u32,
    pub config_bytes: usize,
    /// Порядковый номер элемента — номер порта UART. Значение — вместимость RX FIFO.
    pub uart_tx_ports: u32,
    pub uart_rx_fifo: &'static [usize],
    /// Прикладной предел RX в полёте на одном порту, отдельно от аппаратного FIFO.
    pub uart_rx_in_flight: usize,
    pub i2c_fifo: Option<usize>,
    /// Вместимость буфера SPI2. Номер шины совпадает с номером контроллера.
    pub spi_bytes: Option<usize>,
    pub radio: Option<RadioLimits>,
    /// Все профили имеют модельные ограничения с областью применимости в SUPPORT.md.
    pub blocks_hardware_proof: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RadioLimits {
    pub ports: u32,
    pub packet_bytes: usize,
    pub fifo: usize,
}

pub fn capabilities(profile: BoardId) -> Capabilities {
    use BoardId::*;
    match profile {
        Virtual => Capabilities {
            cpu_hz: sim_mcu::bus::CPU_HZ,
            adc_channels: sim_mcu::periph::adc::CHANNELS as u32,
            config_bytes: sim_mcu::bus::CONFIG_SIZE,
            uart_tx_ports: 1,
            uart_rx_fifo: &[sim_mcu::periph::uart::RX_FIFO_DEPTH],
            uart_rx_in_flight: 0,
            i2c_fifo: None,
            spi_bytes: None,
            radio: Some(RadioLimits {
                ports: 1,
                packet_bytes: sim_mcu::periph::radio::MAX_PACKET,
                fifo: sim_mcu::periph::radio::FIFO_DEPTH,
            }),
            blocks_hardware_proof: true,
        },
        _ => Capabilities {
            cpu_hz: crate::peripheral::Hardware::frequency(profile),
            adc_channels: match profile {
                Rp2040 => 4,
                Stm32f103c8 => 10,
                Atmega328p => 8,
                Esp32C3 => 5,
                Virtual => unreachable!(),
            },
            config_bytes: 0,
            uart_tx_ports: match profile {
                Stm32f103c8 => 3,
                Rp2040 | Esp32C3 => 2,
                Atmega328p => 1,
                Virtual => unreachable!(),
            },
            uart_rx_fifo: if profile == Esp32C3 { &[128, 128] } else { &[] },
            uart_rx_in_flight: if profile == Esp32C3 { 128 } else { 0 },
            i2c_fifo: (profile == Esp32C3).then_some(32),
            spi_bytes: (profile == Esp32C3).then_some(64),
            radio: (profile == Esp32C3).then_some(RadioLimits {
                ports: EspRadioPort::EspNow as u32 + 1,
                packet_bytes: EspRadioPort::Wifi.max_bytes(),
                // ABI адаптера C3: один непрочитанный пакет на порт.
                fifo: 1,
            }),
            blocks_hardware_proof: true,
        },
    }
}

pub(crate) fn event_time(expected: u64, actual: u64) -> Result<(), BoardFault> {
    if expected != actual {
        return Err(BoardFault::Misconfigured(format!(
            "вход в такте {actual}, плата в такте {expected}"
        )));
    }
    Ok(())
}

pub(crate) fn end_cycle(start: u64, count: u64) -> Result<u64, BoardFault> {
    // Та же граница, что у старого run_for. Оставляет место для промежуточных
    // вычислений делителей и завершения начатой инструкции.
    start
        .checked_add(count)
        .filter(|&n| n < (1 << 53))
        .ok_or_else(|| {
            BoardFault::Misconfigured("интервал вне точного диапазона счётчика тактов".into())
        })
}
