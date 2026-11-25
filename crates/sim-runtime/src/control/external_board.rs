//! ESP32-S3 в отдельном процессе QEMU Espressif.
//!
//! UART0 доступен для диагностики. Запрос запускает холодную машину и завершает её перед
//! возвратом. Байты UART не имеют меток виртуального времени: `-icount` задаёт часы гостя,
//! а доставка по stdio идёт через планировщик ОС. Пока нет сверенного снимка всего QEMU,
//! включая устройства, носители и границу обмена, результат опыта открыт.

use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use sim_process::Analysis;

/// Перечень подключённых портов адаптера. UART1/2, монитор, GDB и сеть не подключены.
/// QEMU Espressif, руководство S3 «Run QEMU» и `hw/xtensa/esp32s3.c`, `serial_hd(0)`.
pub const EXTERNAL_BOARD_PORTS: &[&str] = &["uart0"];

/// Документированное покрытие машины QEMU. Оно описывает наличие модели, а не
/// доступность порта в адаптере или полноту реализации блока.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BlockCoverage {
    Documented,
    NotAnalyzed { reason: &'static str },
}

pub const ESP32S3_QEMU_FEATURES: &str =
    "https://github.com/espressif/esp-toolchain-docs/blob/main/qemu/README.md#supported-features";

// Колонка ESP32-S3 таблицы «Supported features» (ESP32S3_QEMU_FEATURES).
const DOCUMENTED: &[&str] = &[
    "dual_core",
    "uart",
    "interrupt_matrix",
    "gpio_strap",
    "spi_flash",
    "flash_mmu",
    "flash_encryption",
    "psram_qpi",
    "psram_opi",
    "psram_mmu",
    "efuse",
    "rng",
    "gdma",
    "aes",
    "sha",
    "rsa",
    "hmac",
    "digital_signature",
    "systimer",
    "timer_groups",
    "twai",
    "virtual_ethernet",
    "virtual_framebuffer",
];
const UNSUPPORTED: &[&str] = &[
    "sdmmc",
    "ledc",
    "wifi",
    "bluetooth",
    "usb",
    "rmt",
    "spi",
    "i2c",
    "i2s",
    "ulp",
    "gpio_matrix",
    "io_mux",
    "direct_boot",
    "secure_boot",
];
const UNDOCUMENTED: &[&str] = &[
    // ESP32-S3 Datasheet §4.1 «System» и §4.2 «Peripherals». Наличие модели RTC-регистров
    // и виртуального RGB-буфера само по себе не подтверждает эти аппаратные блоки и режимы.
    "adc",
    "adc_calibration",
    "pcnt",
    "mcpwm",
    "temperature_sensor",
    "touch",
    "camera",
    "lcd",
    "gpio",
    "rtc",
    "rtc_wdt",
    "clock_tree",
    "modem_sleep",
    "light_sleep",
    "deep_sleep",
    "timer_wake",
    "gpio_wake",
    "brownout",
    "power",
    "esp_now",
    "usb_serial_jtag",
];

/// Полный перечень известных адаптеру идентификаторов блоков в стабильном порядке.
pub fn esp32s3_blocks() -> impl Iterator<Item = (&'static str, BlockCoverage)> {
    DOCUMENTED
        .iter()
        .chain(UNSUPPORTED)
        .chain(UNDOCUMENTED)
        .map(|&block| (block, esp32s3_coverage(block)))
}

/// Неизвестные идентификаторы и недокументированные режимы остаются открыты.
pub fn esp32s3_coverage(block: &str) -> BlockCoverage {
    if DOCUMENTED.contains(&block) {
        return BlockCoverage::Documented;
    }
    let reason = match block {
        // Руководство QEMU Espressif для S3, «Compiling the ESP-IDF program to emulate».
        "secure_boot" => "Secure Boot не поддержан в руководстве QEMU Espressif для ESP32-S3",
        _ if UNSUPPORTED.contains(&block) => {
            "блок не поддержан в таблице возможностей QEMU Espressif для ESP32-S3"
        }
        _ => "поддержка блока не подтверждена таблицей возможностей QEMU Espressif для ESP32-S3",
    };
    BlockCoverage::NotAnalyzed { reason }
}

#[derive(Clone, Debug)]
pub enum Boot {
    /// Готовый объединённый raw flash с загрузчиком и таблицей разделов.
    Flash(PathBuf),
    /// Режим UART0 из ESP-IDF; для запросов SPI можно подключить объединённый образ.
    UartDownload { flash: Option<PathBuf> },
}

#[derive(Clone, Copy, Debug)]
pub struct Psram {
    pub mib: u32,
    pub octal: bool,
}

#[derive(Clone, Copy, Debug)]
pub enum GuestClock {
    /// 2^shift виртуальных наносекунд на инструкцию. QEMU, «Invocation», `-icount`.
    InstructionCount { shift: u8 },
    /// Диагностика совместимости, если прошивка не проходит в режиме icount.
    Diagnostic,
}

#[derive(Clone, Copy, Debug)]
pub struct Limits {
    /// Сторож процесса по монотонным часам хоста. Время гостя из него не выводится.
    pub wall_time: Duration,
    /// Жёсткий RLIMIT_AS процесса в байтах, включая виртуальные отображения ОС.
    pub address_space_bytes: u64,
    pub uart_bytes: usize,
    pub stderr_bytes: usize,
}

#[derive(Clone, Debug)]
pub struct Config {
    pub qemu: PathBuf,
    pub boot: Boot,
    pub efuse: Option<PathBuf>,
    pub psram: Option<Psram>,
    pub clock: GuestClock,
    pub seed: u64,
    pub limits: Limits,
}

#[derive(Clone, Copy, Debug)]
pub struct UartTransmission<'a> {
    /// Послать байты только после получения этой последовательности от гостя.
    pub after: &'a [u8],
    pub bytes: &'a [u8],
}

#[derive(Clone, Copy, Debug)]
pub struct UartProbe<'a> {
    pub port: &'a str,
    pub transmit: Option<UartTransmission<'a>>,
    /// Упорядоченная последовательность, завершающая диагностику. При передаче
    /// поиск начинается среди байтов, прочитанных после записи входа в канал ОС.
    pub until: &'a [u8],
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Stop {
    Marker,
    Deadline,
    UartLimit,
    StderrLimit,
    ChannelsClosed,
    Exited(std::process::ExitStatus),
    Unavailable(String),
    Io(String),
}

#[derive(Debug)]
pub struct Report {
    pub stop: Stop,
    pub uart: Vec<u8>,
    pub stderr: Vec<u8>,
    /// Число байтов, принятых каналом ОС. Подтверждение приёма гостем даёт его протокол.
    pub input_written: usize,
}

impl Report {
    /// Результат, пригодный для передачи в проверку повторяемости. Транскрипт
    /// одного запуска не заменяет снимок, восстановление и проверку продолжения.
    pub fn repeatability(&self) -> Analysis<()> {
        let reason = match &self.stop {
            Stop::Marker => {
                "нет подтверждённого снимка QEMU и привязки UART к виртуальному времени".into()
            }
            Stop::Deadline => "истёк лимит времени процесса QEMU".into(),
            Stop::UartLimit => "исчерпан буфер UART0".into(),
            Stop::StderrLimit => "исчерпан буфер диагностики QEMU".into(),
            Stop::ChannelsClosed => "QEMU закрыл каналы до получения ответа UART0".into(),
            Stop::Exited(status) => format!("QEMU завершился до получения ответа UART0 ({status})"),
            Stop::Unavailable(reason) | Stop::Io(reason) => reason.clone(),
        };
        Analysis::NotAnalyzed { reason }
    }
}

/// Проверяет конфигурацию и выполняет одну ограниченную диагностическую транзакцию.
/// Ошибки входных данных возвращаются сразу, недоступность процесса видна в отчёте.
pub fn probe_esp32s3(config: &Config, probe: UartProbe<'_>) -> Result<Report, String> {
    if !EXTERNAL_BOARD_PORTS.contains(&probe.port) {
        return Err(format!(
            "порт «{}» не подключён, доступен uart0",
            probe.port
        ));
    }
    let limits = config.limits;
    if limits.wall_time.is_zero()
        || limits.address_space_bytes == 0
        || limits.uart_bytes == 0
        || limits.stderr_bytes == 0
    {
        return Err("лимиты QEMU должны быть положительными".into());
    }
    // sys/resource.h Darwin: RLIM_INFINITY = INT64_MAX. Профиль использует
    // конечный лимит на всех поддерживаемых ОС.
    if limits.address_space_bytes >= i64::MAX as u64 {
        return Err("нужен конечный лимит адресного пространства QEMU".into());
    }
    let ready_len = if let Some(tx) = probe.transmit {
        if tx.after.is_empty() || tx.bytes.is_empty() {
            return Err("для передачи UART0 нужны непустые маркер готовности и данные".into());
        }
        tx.after.len()
    } else {
        0
    };
    if probe.until.is_empty() || ready_len.saturating_add(probe.until.len()) > limits.uart_bytes {
        return Err("непустые маркеры UART0 должны помещаться в лимит приёма".into());
    }
    if config.qemu.as_os_str().is_empty() {
        return Err("не задан исполняемый файл QEMU".into());
    }
    // QEMU Espressif, accel/tcg/icount-common.c, MAX_ICOUNT_SHIFT.
    if matches!(config.clock, GuestClock::InstructionCount { shift } if shift > 10) {
        return Err("shift для -icount должен быть в диапазоне 0–10".into());
    }
    // Руководство QEMU Espressif для S3, «Adding PSRAM».
    if config
        .psram
        .is_some_and(|psram| ![2, 4, 8, 16, 32].contains(&psram.mib))
    {
        return Err("QEMU для ESP32-S3 поддерживает PSRAM 2, 4, 8, 16 или 32 МиБ".into());
    }

    // Руководство QEMU Espressif для S3, «Compiling the ESP-IDF program to emulate» и
    // «Specifying eFuse storage». Размеры именно образов QEMU, а не регистровых окон.
    const KIB: u64 = 1 << 10;
    const MIB: u64 = 1 << 20;
    let flash = match &config.boot {
        Boot::Flash(path) => Some(path),
        Boot::UartDownload { flash } => flash.as_ref(),
    };
    let flash = flash
        .map(|path| image(path, &[2 * MIB, 4 * MIB, 8 * MIB, 16 * MIB]))
        .transpose()?;
    let efuse = config
        .efuse
        .as_ref()
        .map(|path| image(path, &[KIB]))
        .transpose()?;
    let deadline = Instant::now()
        .checked_add(limits.wall_time)
        .ok_or("лимит времени процесса QEMU вне диапазона часов хоста")?;
    Ok(host::run(
        config,
        flash.as_deref(),
        efuse.as_deref(),
        probe,
        deadline,
    ))
}

// Канонический путь гарантирует локальный file backend QEMU. Двойная запятая экранирует
// разделитель опций согласно документации QEMU «Invocation», раздел «Block device options».
fn image(path: &Path, sizes: &[u64]) -> Result<String, String> {
    let error = |error: std::io::Error| format!("образ {}: {error}", path.display());
    let path = path.canonicalize().map_err(error)?;
    let metadata = path.metadata().map_err(error)?;
    if !metadata.is_file() || !sizes.contains(&metadata.len()) {
        return Err(format!(
            "образ {} должен быть обычным файлом размером в байтах из {sizes:?}",
            path.display()
        ));
    }
    let path = path
        .to_str()
        .ok_or("путь к образу QEMU должен быть UTF-8")?;
    Ok(path.replace(',', ",,"))
}

#[cfg(any(
    target_os = "macos",
    all(
        target_os = "linux",
        any(
            target_arch = "x86_64",
            target_arch = "aarch64",
            target_arch = "riscv64"
        )
    )
))]
mod host {
    use super::*;
    use std::io::{self, Read, Write};
    use std::os::fd::{AsRawFd, OwnedFd};
    use std::os::unix::{net::UnixStream, process::CommandExt};
    use std::process::{Child, Command, Stdio};

    pub(super) fn run(
        config: &Config,
        flash: Option<&str>,
        efuse: Option<&str>,
        probe: UartProbe<'_>,
        deadline: Instant,
    ) -> Report {
        let mut report = Report {
            stop: Stop::ChannelsClosed,
            uart: Vec::new(),
            stderr: Vec::new(),
            input_written: 0,
        };
        report.stop = match start(config, flash, efuse) {
            Ok((mut child, uart, stderr)) => {
                let stop = exchange(&uart, &stderr, deadline, config.limits, probe, &mut report);
                let stop = match stop {
                    Ok(Stop::ChannelsClosed) => child
                        .0
                        .try_wait()
                        .map(|status| status.map_or(Stop::ChannelsClosed, Stop::Exited)),
                    other => other,
                };
                stop.unwrap_or_else(|error| Stop::Io(format!("обмен с QEMU: {error}")))
            }
            Err(error) => Stop::Unavailable(format!(
                "запуск QEMU с лимитом адресного пространства: {error}"
            )),
        };
        report
    }

    fn start(
        config: &Config,
        flash: Option<&str>,
        efuse: Option<&str>,
    ) -> io::Result<(Worker, UnixStream, UnixStream)> {
        let (uart, guest_uart) = UnixStream::pair()?;
        let (stderr, guest_stderr) = UnixStream::pair()?;
        uart.set_nonblocking(true)?;
        stderr.set_nonblocking(true)?;

        let mut qemu = Command::new(&config.qemu);
        qemu
            // QEMU Invocation: headless, без TCP/UDP/монитора и без дополнительных устройств.
            .args([
                "-M",
                "esp32s3",
                "-accel",
                "tcg,thread=single",
                "-nodefaults",
                "-nic",
                "none",
                "-display",
                "none",
                "-monitor",
                "none",
                "-chardev",
                "stdio,id=uart0,signal=off",
                "-serial",
                "chardev:uart0",
                // QEMU «Invocation», -rtc: начало POSIX Epoch, часы внутри гостя.
                "-rtc",
                "base=1970-01-01T00:00:00,clock=vm",
            ])
            // В ESP32-S3 два ядра (Datasheet §4.1.1); оставляем топологию машины QEMU.
            // Руководство QEMU S3, «Adding PSRAM»: -m здесь задаёт PSRAM, 0 означает её отсутствие.
            .arg("-m")
            .arg(
                config
                    .psram
                    .map_or_else(|| "0".into(), |psram| format!("{}M", psram.mib)),
            )
            // QEMU Invocation, -seed; ESP32-S3 RNG использует hw/misc/esp32s3_rng.c.
            .arg("-seed")
            .arg(
                sim_signal::Rng::stream(config.seed, "external_board/esp32s3")
                    .next_u64()
                    .to_string(),
            )
            .stdin(Stdio::from(OwnedFd::from(guest_uart.try_clone()?)))
            .stdout(Stdio::from(OwnedFd::from(guest_uart)))
            .stderr(Stdio::from(OwnedFd::from(guest_stderr)));

        if let GuestClock::InstructionCount { shift } = config.clock {
            qemu.arg("-icount")
                .arg(format!("shift={shift},sleep=off,align=off"));
        }
        // Flash: руководство QEMU S3, «Specifying bootstrapping mode». UART0: ESP-IDF
        // tools/idf_py_actions/qemu_ext.py, QEMU_TARGETS['esp32s3']. Штатный UART0-only boot.
        let strap = match config.boot {
            Boot::Flash(_) => "0x04",
            Boot::UartDownload { .. } => "0x07",
        };
        qemu.arg("-global").arg(format!(
            "driver=esp32s3.gpio,property=strap_mode,value={strap}"
        ));
        // snapshot=on создаёт временную накладку записи QEMU, не снимок машины.
        if let Some(flash) = flash {
            qemu.arg("-drive")
                .arg(format!("file={flash},if=mtd,format=raw,snapshot=on"));
        }
        if let Some(efuse) = efuse {
            qemu.arg("-drive").arg(format!(
                "file={efuse},if=none,format=raw,id=efuse,snapshot=on"
            ));
            qemu.args([
                "-global",
                "driver=nvram.esp32s3.efuse,property=drive,value=efuse",
            ]);
        }
        // Руководство QEMU Espressif для S3, «Enabling octal PSRAM».
        if config.psram.is_some_and(|psram| psram.octal) {
            qemu.args(["-global", "driver=ssi_psram,property=is_octal,value=true"]);
        }

        let bytes = config.limits.address_space_bytes;
        // SAFETY: после fork вызываем только POSIX getrlimit/setrlimit, до запуска QEMU.
        unsafe { qemu.pre_exec(move || os::set_limits(bytes)) };
        Ok((Worker(qemu.spawn()?), uart, stderr))
    }

    struct Worker(Child);

    impl Drop for Worker {
        fn drop(&mut self) {
            // У QEMU нет дочерних внешних программ с заданными здесь аргументами.
            // wait забирает статус даже после SIGKILL либо досрочного завершения.
            let _ = self.0.kill();
            let _ = self.0.wait();
        }
    }

    fn exchange(
        mut uart: &UnixStream,
        mut stderr: &UnixStream,
        deadline: Instant,
        limits: Limits,
        probe: UartProbe<'_>,
        report: &mut Report,
    ) -> io::Result<Stop> {
        let mut ready = probe.transmit.is_none();
        let input = probe.transmit.map_or(&[][..], |tx| tx.bytes);
        let mut scan_from = 0;
        let mut uart_open = true;
        let mut stderr_open = true;
        // 4 КиБ — размер чтения диагностики, не параметр эмулируемой платы.
        let mut chunk = [0; 4096];
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            if left.is_zero() {
                return Ok(Stop::Deadline);
            }
            if !uart_open && !stderr_open {
                return Ok(Stop::ChannelsClosed);
            }
            let writing = uart_open && ready && report.input_written < input.len();
            let [uart_io, stderr_io] = os::wait(
                uart_open.then_some(uart.as_raw_fd()),
                stderr_open.then_some(stderr.as_raw_fd()),
                writing,
                left,
            )?;
            if Instant::now() >= deadline {
                return Ok(Stop::Deadline);
            }

            if stderr_io.readable {
                let remaining = limits.stderr_bytes - report.stderr.len();
                let count = remaining.min(chunk.len());
                match stderr.read(&mut chunk[..count]) {
                    Ok(0) => stderr_open = false,
                    Ok(count) => {
                        report.stderr.extend_from_slice(&chunk[..count]);
                        if report.stderr.len() == limits.stderr_bytes {
                            return Ok(Stop::StderrLimit);
                        }
                    }
                    Err(error) if retryable(&error) => {}
                    Err(error) => return Err(error),
                }
            }
            if uart_io.readable {
                let remaining = limits.uart_bytes - report.uart.len();
                let count = remaining.min(chunk.len());
                match uart.read(&mut chunk[..count]) {
                    Ok(0) => uart_open = false,
                    Ok(count) => {
                        report.uart.extend_from_slice(&chunk[..count]);
                        if !ready {
                            let marker = probe
                                .transmit
                                .expect("ожидание маркера перед передачей")
                                .after;
                            ready = marker_end(&report.uart, marker, &mut scan_from).is_some();
                        } else if !writing
                            && let Some(end) = marker_end(&report.uart, probe.until, &mut scan_from)
                        {
                            report.uart.truncate(end);
                            return Ok(Stop::Marker);
                        }
                        if report.uart.len() == limits.uart_bytes {
                            return Ok(Stop::UartLimit);
                        }
                    }
                    Err(error) if retryable(&error) => {}
                    Err(error) => return Err(error),
                }
            }
            if uart_io.writable && uart_open {
                match uart.write(&input[report.input_written..]) {
                    Ok(0) => return Err(io::ErrorKind::WriteZero.into()),
                    Ok(count) => {
                        report.input_written += count;
                        scan_from = report.uart.len();
                    }
                    Err(error) if retryable(&error) => {}
                    Err(error) => return Err(error),
                }
            }
        }
    }

    fn marker_end(bytes: &[u8], marker: &[u8], from: &mut usize) -> Option<usize> {
        let end = bytes[*from..]
            .windows(marker.len())
            .position(|window| window == marker)
            .map(|index| *from + index + marker.len());
        // Оставить лишь те начала, для которых ещё не пришла полная длина маркера.
        *from = (*from).max(bytes.len().saturating_sub(marker.len() - 1));
        end
    }

    fn retryable(error: &io::Error) -> bool {
        matches!(
            error.kind(),
            io::ErrorKind::WouldBlock | io::ErrorKind::Interrupted
        )
    }

    mod os {
        use std::ffi::{c_int, c_short};
        use std::io;
        use std::time::Duration;

        // POSIX poll. Типы nfds_t и номера RLIMIT_* взяты из sys/poll.h и
        // sys/resource.h Darwin, либо poll.h и Linux UAPI asm-generic/resource.h.
        #[cfg(target_os = "macos")]
        type Nfds = u32;
        #[cfg(target_os = "linux")]
        type Nfds = usize;
        #[cfg(target_os = "macos")]
        const RLIMIT_AS: c_int = 5;
        #[cfg(target_os = "linux")]
        const RLIMIT_AS: c_int = 9;
        const RLIMIT_CORE: c_int = 4;
        const POLLIN: c_short = 0x01;
        const POLLOUT: c_short = 0x04;
        const POLLERR: c_short = 0x08;
        const POLLHUP: c_short = 0x10;
        const POLLNVAL: c_short = 0x20;

        #[repr(C)]
        struct PollFd {
            fd: c_int,
            events: c_short,
            revents: c_short,
        }

        #[repr(C)]
        struct Rlimit {
            current: u64,
            maximum: u64,
        }

        unsafe extern "C" {
            fn poll(fds: *mut PollFd, len: Nfds, timeout: c_int) -> c_int;
            fn getrlimit(resource: c_int, limit: *mut Rlimit) -> c_int;
            fn setrlimit(resource: c_int, limit: *const Rlimit) -> c_int;
        }

        #[derive(Default)]
        pub(super) struct Events {
            pub readable: bool,
            pub writable: bool,
        }

        pub(super) fn wait(
            uart: Option<c_int>,
            stderr: Option<c_int>,
            write: bool,
            left: Duration,
        ) -> io::Result<[Events; 2]> {
            let mut descriptors = [uart, stderr].map(|fd| PollFd {
                fd: fd.unwrap_or(-1),
                events: POLLIN,
                revents: 0,
            });
            if write {
                descriptors[0].events |= POLLOUT;
            }
            // POSIX poll использует целые миллисекунды; округляем вверх.
            let timeout = (left.as_nanos().div_ceil(1_000_000)).min(c_int::MAX as u128) as c_int;
            // SAFETY: передаём доступный на запись массив ABI pollfd, длина соответствует.
            if unsafe { poll(descriptors.as_mut_ptr(), descriptors.len() as Nfds, timeout) } < 0 {
                let error = io::Error::last_os_error();
                if error.kind() == io::ErrorKind::Interrupted {
                    return Ok([Events::default(), Events::default()]);
                }
                return Err(error);
            }
            if descriptors.iter().any(|fd| fd.revents & POLLNVAL != 0) {
                return Err(io::ErrorKind::InvalidInput.into());
            }
            Ok(descriptors.map(|fd| Events {
                readable: fd.revents & (POLLIN | POLLERR | POLLHUP) != 0,
                writable: fd.events & POLLOUT != 0 && fd.revents & POLLOUT != 0,
            }))
        }

        pub(super) fn set_limits(bytes: u64) -> io::Result<()> {
            let mut limit = Rlimit {
                current: 0,
                maximum: 0,
            };
            // SAFETY: указатели ссылаются на инициализированную структуру ABI rlimit.
            if unsafe { getrlimit(RLIMIT_AS, &mut limit) } != 0 {
                return Err(io::Error::last_os_error());
            }
            // Не ослабляем унаследованные ограничения. Запрещаем core dump QEMU.
            let cap = bytes.min(limit.current).min(limit.maximum);
            let address = Rlimit {
                current: cap,
                maximum: cap,
            };
            let core = Rlimit {
                current: 0,
                maximum: 0,
            };
            if unsafe { setrlimit(RLIMIT_AS, &address) } != 0
                || unsafe { setrlimit(RLIMIT_CORE, &core) } != 0
            {
                return Err(io::Error::last_os_error());
            }
            Ok(())
        }
    }
}

#[cfg(not(any(
    target_os = "macos",
    all(
        target_os = "linux",
        any(
            target_arch = "x86_64",
            target_arch = "aarch64",
            target_arch = "riscv64"
        )
    )
)))]
mod host {
    use super::*;

    pub(super) fn run(
        _: &Config,
        _: Option<&str>,
        _: Option<&str>,
        _: UartProbe<'_>,
        _: Instant,
    ) -> Report {
        Report {
            stop: Stop::Unavailable("для этой ОС нет ограничителя памяти QEMU".into()),
            uart: Vec::new(),
            stderr: Vec::new(),
            input_written: 0,
        }
    }
}
