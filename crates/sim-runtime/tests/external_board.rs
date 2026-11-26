use sim_process::Analysis;
use sim_runtime::control::external_board::{
    BlockCoverage, Boot, Config, EXTERNAL_BOARD_PORTS, GuestClock, Limits, Psram, Report, Stop,
    UartProbe, UartTransmission, esp32s3_blocks, esp32s3_coverage, probe_esp32s3,
};
use std::path::PathBuf;
use std::time::Duration;

fn config() -> Config {
    Config {
        qemu: "/несуществующий/qemu-system-xtensa".into(),
        boot: Boot::UartDownload { flash: None },
        efuse: None,
        psram: None,
        // 2^3 виртуальных нс/инструкцию — тестовая сетка, не частота чипа.
        clock: GuestClock::InstructionCount { shift: 3 },
        seed: 0,
        limits: Limits {
            // Бюджеты тестового процесса и диагностических буферов. На macOS RLIMIT_AS
            // включает зарезервированную dyld общую область (~466 ГиБ на стенде arm64).
            wall_time: Duration::from_secs(5),
            address_space_bytes: if cfg!(target_os = "macos") {
                480 << 30
            } else {
                2 << 30
            },
            uart_bytes: 16 << 10,
            stderr_bytes: 4 << 10,
        },
    }
}

fn listen(until: &[u8]) -> UartProbe<'_> {
    UartProbe {
        port: "uart0",
        transmit: None,
        until,
    }
}

fn open(report: &Report) -> String {
    match report.repeatability() {
        Analysis::NotAnalyzed { reason } => reason,
        Analysis::Analyzed { .. } => panic!("внешний процесс без снимка принят как повторяемый"),
    }
}

#[test]
fn rejects_incompatible_configuration_and_unknown_port() {
    let mut config = config();
    let mut probe = listen("ответ".as_bytes());
    probe.port = "uart1";
    assert!(probe_esp32s3(&config, probe).unwrap_err().contains("порт"));
    assert_eq!(EXTERNAL_BOARD_PORTS, &["uart0"]);

    assert!(
        probe_esp32s3(&config, listen(b""))
            .unwrap_err()
            .contains("маркеры")
    );
    let mut probe = listen(b"ok");
    probe.transmit = Some(UartTransmission {
        after: b"",
        bytes: b"?",
    });
    assert!(
        probe_esp32s3(&config, probe)
            .unwrap_err()
            .contains("готовности")
    );

    config.clock = GuestClock::InstructionCount { shift: 11 };
    assert!(
        probe_esp32s3(&config, listen(b"ok"))
            .unwrap_err()
            .contains("shift")
    );
    config.clock = GuestClock::Diagnostic;
    config.psram = Some(Psram {
        mib: 3,
        octal: false,
    });
    assert!(
        probe_esp32s3(&config, listen(b"ok"))
            .unwrap_err()
            .contains("PSRAM")
    );
    config.psram = None;
    config.limits.address_space_bytes = 0;
    assert!(
        probe_esp32s3(&config, listen(b"ok"))
            .unwrap_err()
            .contains("лимиты")
    );
    config.limits.address_space_bytes = u64::MAX;
    assert!(
        probe_esp32s3(&config, listen(b"ok"))
            .unwrap_err()
            .contains("конечный")
    );
    config.limits.address_space_bytes = 1;
    config.limits.wall_time = Duration::MAX;
    assert!(
        probe_esp32s3(&config, listen(b"ok"))
            .unwrap_err()
            .contains("вне диапазона")
    );
}

#[test]
fn missing_qemu_keeps_repeatability_open() {
    let report = probe_esp32s3(&config(), listen(b"ok")).unwrap();
    assert!(matches!(report.stop, Stop::Unavailable(_)));
    assert!(report.uart.is_empty());
    assert_eq!(report.input_written, 0);
    assert!(!open(&report).is_empty());
}

#[test]
fn coverage_refuses_unsupported_peripherals_individually() {
    // Строки ❌ либо N/A в колонке ESP32-S3 espressif/esp-toolchain-docs/qemu/README.md,
    // Secure Boot в qemu/esp32s3/README.md, остальные режимы не заявлены.
    let blocks: std::collections::BTreeMap<_, _> = esp32s3_blocks().collect();
    assert_eq!(blocks.len(), esp32s3_blocks().count());
    for block in [
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
        "camera",
        "adc",
        "deep_sleep",
        "gpio_wake",
        "timer_wake",
        "esp_now",
    ] {
        assert!(
            matches!(blocks.get(block), Some(BlockCoverage::NotAnalyzed { reason }) if !reason.is_empty()),
            "{block}"
        );
    }
    for block in ["uart", "spi_flash", "gpio_strap", "gdma", "systimer"] {
        assert_eq!(
            blocks.get(block),
            Some(&BlockCoverage::Documented),
            "{block}"
        );
    }
    assert!(matches!(
        esp32s3_coverage("неизвестный_блок"),
        BlockCoverage::NotAnalyzed { .. }
    ));
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
mod supervised {
    use super::*;
    use std::fs::{self, File};
    use std::os::unix::fs::PermissionsExt;
    use std::time::Instant;

    struct Fixture(PathBuf);

    impl Fixture {
        fn new(name: &str) -> Self {
            let path = std::env::temp_dir()
                .join(format!("sim-external-board-{}-{name}", std::process::id()));
            fs::create_dir(&path).unwrap();
            Self(path)
        }

        fn script(&self, program: &str) -> PathBuf {
            let path = self.0.join("qemu-fixture");
            fs::write(&path, format!("#!/bin/sh\n{program}\n")).unwrap();
            fs::set_permissions(&path, fs::Permissions::from_mode(0o700)).unwrap();
            path
        }
    }

    impl Drop for Fixture {
        fn drop(&mut self) {
            fs::remove_dir_all(&self.0).unwrap();
        }
    }

    fn assert_reaped(pid: &str) {
        unsafe extern "C" {
            fn kill(pid: i32, signal: i32) -> i32;
        }
        let pid = pid.parse::<i32>().unwrap();
        assert!(pid > 0);
        // POSIX kill с сигналом 0 только проверяет существование процесса. ESRCH = 3
        // в sys/errno.h Darwin и Linux UAPI asm-generic/errno-base.h.
        assert_eq!(unsafe { kill(pid, 0) }, -1);
        assert_eq!(std::io::Error::last_os_error().raw_os_error(), Some(3));
    }

    #[test]
    fn exchanges_bytes_and_reaps_without_listening_on_tcp() {
        let fixture = Fixture::new("exchange");
        let mut config = config();
        config.qemu = fixture.script(
            r#"
printf '%s\n' "$@" >&2
printf '%s READY\n' "$$"
IFS= read -r value
printf 'RX=%s END surplus\n' "$value"
while :; do :; done
"#,
        );
        let mut probe = listen(b"END");
        probe.transmit = Some(UartTransmission {
            after: b"READY\n",
            bytes: b"abc\n",
        });
        let report = probe_esp32s3(&config, probe).unwrap();
        assert_eq!(report.stop, Stop::Marker, "{report:?}");
        assert_eq!(report.input_written, 4);
        let uart = std::str::from_utf8(&report.uart).unwrap();
        let (pid, result) = uart.split_once(' ').unwrap();
        assert_eq!(result, "READY\nRX=abc END");
        assert_reaped(pid);
        assert!(open(&report).contains("снимка"));
        let args: Vec<_> = std::str::from_utf8(&report.stderr)
            .unwrap()
            .lines()
            .collect();
        assert!(args.windows(2).any(|arg| arg == ["-nic", "none"]));
        assert!(args.windows(2).any(|arg| arg == ["-monitor", "none"]));
        assert!(
            args.windows(2)
                .any(|arg| arg == ["-serial", "chardev:uart0"])
        );
        assert!(
            args.windows(2)
                .any(|arg| arg == ["-icount", "shift=3,sleep=off,align=off"])
        );
        assert!(
            !args
                .iter()
                .any(|arg| arg.contains("tcp:") || *arg == "-gdb")
        );
    }

    #[test]
    fn stops_on_deadline_and_refuses_output_before_request() {
        let fixture = Fixture::new("deadline");
        let mut config = config();
        // Включает запуск интерпретатора под параллельной нагрузкой cargo.
        config.limits.wall_time = Duration::from_secs(2);
        config.qemu = fixture.script(
            r#"
printf '%s MATCH\n' "$$"
IFS= read -r request
while :; do :; done
"#,
        );
        let mut probe = listen(b"MATCH");
        probe.transmit = Some(UartTransmission {
            after: b"unreachable",
            bytes: b"input\n",
        });
        let started = Instant::now();
        let report = probe_esp32s3(&config, probe).unwrap();
        assert_eq!(report.stop, Stop::Deadline, "{report:?}");
        assert!(started.elapsed() >= config.limits.wall_time);
        assert!(
            started.elapsed() < Duration::from_secs(5),
            "сторож оставил процесс"
        );
        assert_eq!(report.input_written, 0);
        assert_reaped(
            std::str::from_utf8(&report.uart)
                .unwrap()
                .split_once(' ')
                .unwrap()
                .0,
        );
        assert!(open(&report).contains("лимит времени"));
    }

    #[test]
    fn stops_on_each_capture_limit_and_reaps() {
        let fixture = Fixture::new("output-limit");
        let mut config = config();
        config.limits.uart_bytes = 48;
        config.limits.stderr_bytes = 48;
        for (redirect, expected) in [("", Stop::UartLimit), (">&2", Stop::StderrLimit)] {
            config.qemu = fixture.script(&format!(
                "printf '%s ' \"$$\" {redirect}\nwhile :; do printf '0123456789' {redirect}; done"
            ));
            let report = probe_esp32s3(&config, listen(b"end")).unwrap();
            assert_eq!(report.stop, expected, "{report:?}");
            let buffer = if expected == Stop::UartLimit {
                &report.uart
            } else {
                &report.stderr
            };
            assert_eq!(buffer.len(), 48);
            assert_reaped(
                std::str::from_utf8(buffer)
                    .unwrap()
                    .split_once(' ')
                    .unwrap()
                    .0,
            );
            assert!(open(&report).contains("исчерпан"));
        }
    }

    #[test]
    fn enforces_address_space_limit_before_guest_code() {
        let fixture = Fixture::new("memory-limit");
        let mut config = config();
        config.qemu = fixture.script("printf 'код запущен\n'");
        config.limits.address_space_bytes = 1;
        let report = probe_esp32s3(&config, listen("код запущен".as_bytes())).unwrap();
        assert!(
            matches!(
                report.stop,
                Stop::Unavailable(_) | Stop::Exited(_) | Stop::ChannelsClosed
            ),
            "{report:?}"
        );
        assert!(report.uart.is_empty());
        assert!(!open(&report).is_empty());
    }

    #[test]
    fn validates_image_sizes_and_uses_private_writes() {
        let fixture = Fixture::new("images");
        let mut config = config();
        config.qemu = fixture.script("printf '%s\n' \"$@\" >&2\nprintf 'ok\n'");
        let flash = fixture.0.join("flash,name.bin");
        let efuse = fixture.0.join("efuse.bin");
        config.boot = Boot::Flash(flash.clone());
        config.efuse = Some(efuse.clone());
        fs::write(&flash, [0; 8]).unwrap();
        fs::write(&efuse, [0; 8]).unwrap();
        assert!(
            probe_esp32s3(&config, listen(b"ok"))
                .unwrap_err()
                .contains("flash")
        );
        // Допустимые размеры из руководства S3 «Compiling…», «Specifying eFuse storage».
        File::options()
            .write(true)
            .open(&flash)
            .unwrap()
            .set_len(2 << 20)
            .unwrap();
        assert!(
            probe_esp32s3(&config, listen(b"ok"))
                .unwrap_err()
                .contains("efuse")
        );
        File::options()
            .write(true)
            .open(&efuse)
            .unwrap()
            .set_len(1 << 10)
            .unwrap();
        let report = probe_esp32s3(&config, listen(b"ok")).unwrap();
        assert_eq!(report.stop, Stop::Marker);
        let args = std::str::from_utf8(&report.stderr).unwrap();
        assert!(
            args.contains("flash,,name.bin,if=mtd,format=raw,snapshot=on"),
            "{args}"
        );
        assert!(
            args.contains("efuse.bin,if=none,format=raw,id=efuse,snapshot=on"),
            "{args}"
        );
    }

    fn qemu() -> PathBuf {
        std::env::var_os("SIM_EXTERNAL_BOARD_QEMU")
            .expect("задайте путь к QEMU Espressif")
            .into()
    }

    #[test]
    #[ignore = "нужен установленный QEMU Espressif: SIM_EXTERNAL_BOARD_QEMU"]
    fn qemu_uart0_rom() {
        let mut config = config();
        config.qemu = qemu();
        let prompt = b"wait uart0 download\r\n";
        let boot = probe_esp32s3(&config, listen(prompt)).unwrap();
        assert_eq!(boot.stop, Stop::Marker, "{boot:?}");
        assert!(boot.uart.starts_with(b"ESP-ROM:esp32s3-"));
        assert!(open(&boot).contains("виртуальному времени"));

        // Espressif esptool, Serial Protocol: SLIP, SYNC (0x08), заголовки запроса/ответа.
        // Полезная нагрузка SYNC реализована в esptool/loader.py::ESPLoader.sync.
        let mut request = b"\xc0\x00\x08\x24\x00\x00\x00\x00\x00\x07\x07\x12\x20".to_vec();
        request.extend([0x55; 32]);
        request.push(0xc0);
        let expected = b"\xc0\x01\x08\x04\x00\x07\x07\x12\x20\x00\x00\x00\x00\xc0";
        let mut probe = listen(expected);
        probe.transmit = Some(UartTransmission {
            after: prompt,
            bytes: &request,
        });
        config.clock = GuestClock::Diagnostic;
        let first = probe_esp32s3(&config, probe).unwrap();
        let second = probe_esp32s3(&config, probe).unwrap();
        for report in [&first, &second] {
            assert_eq!(report.stop, Stop::Marker, "{report:?}");
            assert_eq!(report.input_written, request.len());
            assert!(report.uart.ends_with(expected));
            assert!(open(report).contains("снимка"));
        }
        assert_eq!(first.uart, second.uart);
        println!(
            "ROM UART0: SYNC {} → {} байт, два байтово равных холодных запуска",
            request.len(),
            expected.len()
        );
    }

    #[test]
    #[ignore = "нужны QEMU Espressif, объединённый образ ESP-IDF hello_world и профиль eFuse"]
    fn qemu_idf_flash() {
        let mut config = config();
        config.qemu = qemu();
        let flash = PathBuf::from(
            std::env::var_os("SIM_EXTERNAL_BOARD_FLASH").expect("нужен объединённый flash"),
        );
        let efuse =
            PathBuf::from(std::env::var_os("SIM_EXTERNAL_BOARD_EFUSE").expect("нужен образ eFuse"));
        let flash_before = fs::read(&flash).unwrap();
        let efuse_before = fs::read(&efuse).unwrap();
        config.boot = Boot::Flash(flash.clone());
        config.efuse = Some(efuse.clone());
        config.clock = GuestClock::Diagnostic;
        let report = probe_esp32s3(&config, listen(b"Hello world!")).unwrap();
        assert_eq!(report.stop, Stop::Marker, "{report:?}");
        assert!(
            report
                .uart
                .windows(b"Calling app_main()".len())
                .any(|w| w == b"Calling app_main()")
        );
        assert!(report.uart.ends_with(b"Hello world!"));
        assert_eq!(fs::read(flash).unwrap(), flash_before);
        assert_eq!(fs::read(efuse).unwrap(), efuse_before);
        assert!(open(&report).contains("снимка"));
        println!(
            "ESP-IDF: flash {} байт, eFuse {} байт, UART0 дошёл до Hello world!",
            flash_before.len(),
            efuse_before.len()
        );
    }
}
