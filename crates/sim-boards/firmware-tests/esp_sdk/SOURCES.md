# Эталонные прогоны ESP32-C3

`hello_world` повторяет штатный пример ESP-IDF v6.1.1 (`051e4c4eb20346e043cdd47bc388fde75ff8af63`). В C-файле переведён комментарий. Настройка `CONFIG_APP_REPRODUCIBLE_BUILD=y` убирает время сборки и абсолютные пути из образа. [Пример ESP-IDF](https://github.com/espressif/esp-idf/tree/v6.1.1/examples/get-started/hello_world).

`Blink` использует `pinMode`, `digitalWrite` и `delay(1000)` из штатного ядра Arduino ESP32 4.0.0. Выбран внешний GPIO3, пригодный для цифрового выхода по таблицам 2-4 и 2-7 даташита. `Blink.ino` добавляет вывод такта FreeRTOS и две метки `esp_timer_get_time()` вокруг `digitalWrite`. Профиль `esp32:esp32:esp32c3:CDCOnBoot=default,FlashMode=dio` задаёт UART0 и штатный DIO-загрузчик. С QIO-загрузчиком в этом QEMU получена ошибка `Failed to set QIE bit` и остановка при инициализации flash. [Даташит ESP32-C3](https://documentation.espressif.com/esp32-c3_datasheet_en.pdf), [Arduino ESP32 4.0.0](https://github.com/espressif/arduino-esp32/tree/4.0.0).

## Воспроизведение

Нужны установленные ESP-IDF v6.1.1, `arduino-cli` (снимался на 1.6.0), Arduino ESP32 4.0.0 и QEMU Espressif `esp-develop-9.2.2-20260417`. Скрипт использует `~/esp/esp-idf` или `SIM_ESP_IDF_PATH`. На macOS добавляет Python 3.13 из Homebrew первым в исходный `PATH`, затем выполняет `export.sh`. Для сборки создаёт временный подкаталог внутри `SIM_ESP_SDK_TMPDIR` и удаляет его при выходе. Для команды Cargo каталог по умолчанию берётся из `std::env::temp_dir()`. [Сборка QEMU Espressif](https://github.com/espressif/qemu/releases/tag/esp-develop-9.2.2-20260417).

Запуск из корня репозитория

```sh
SIM_ESP_SDK_TMPDIR=/tmp bash crates/sim-boards/firmware-tests/esp_sdk/capture.sh hello_world
SIM_ESP_SDK_TMPDIR=/tmp bash crates/sim-boards/firmware-tests/esp_sdk/capture.sh blink
SIM_ESP_SDK_TMPDIR=/tmp cargo test --offline -p sim-boards --test esp_sdk_trace -- --ignored
```

`capture.sh` печатает новую трассу в stdout. Игнорируемый по умолчанию тест пересобирает оба примера последовательно и сравнивает трассы побайтно. Обычный запуск Cargo проверяет содержимое сохранённых трасс без SDK. Для образа ESP-IDF скрипт использует `esptool merge-bin @flash_args`, для Arduino берёт `.merged.bin`, созданный штатным `platform.txt`. `esptool image-info` подтверждает XOR и SHA-256 загрузчика и приложения обоих примеров.

Команда QEMU из `capture.py` запускается в каталоге временной сборки. `efuse.bin` берётся из `QEMU_TARGETS['esp32c3'].default_efuse` в `tools/idf_py_actions/qemu_ext.py` ESP-IDF v6.1.1. SDK задаёт ревизию 0.3 и отключает TIMG WDT для этого запуска QEMU.

```sh
qemu-system-riscv32 -M esp32c3 -nographic -monitor none -serial stdio -nic none \
  -icount shift=3,sleep=off -no-reboot \
  -drive file=flash.bin,if=mtd,format=raw \
  -drive file=efuse.bin,if=none,format=raw,id=efuse \
  -global driver=nvram.esp32c3.efuse,property=drive,value=efuse \
  -global driver=timer.esp32c3.timg,property=wdt_disable,value=true
```

Для Blink добавляется `-trace enable=memory_region_ops_write,file=mmio.log`. Канал stdin остаётся открытым и пустым. `-icount shift=3,sleep=off` даёт виртуальную шкалу 8 нс на инструкцию и прыжок к следующему таймеру при простое. Лимиты времени хоста останавливают процесс при ошибке. [Параметры `icount` QEMU](https://www.qemu.org/docs/master/system/invocation.html#hxtool-15).

## Сверка холодной загрузки BoardSim

Для тестов `esp_sdk_boot` соберите `hello_world` во временном каталоге и оставьте объединённый `flash.bin`. В приведённой команде QEMU замените параметры экрана, монитора и UART на `-display none -monitor stdio -serial file:uart.txt -S`. Добавьте `-accel tcg,one-insn-per-tb=on -d in_asm,exec,nochain -D qemu.log -trace 'enable=memory_region_ops_*'`. До `cont` сохраните снимок eFuse командой монитора `pmemsave 0x6000882c 336 "readback.bin"`. QEMU завершится по сбросу из приложения.

Передайте абсолютные пути к входам через `SIM_ESP_BOOT_FLASH` (сырой flash), `SIM_ESP_BOOT_ROM` (полный `esp32c3-rom.bin` rev. 3), `SIM_ESP_BOOT_READBACK`, `SIM_ESP_BOOT_QEMU_UART`, `SIM_ESP_BOOT_QEMU_PC` и `SIM_ESP_BOOT_QEMU_MMIO` (последние две указывают на `qemu.log`). `SIM_ESP_BOOT_PC_APP_MAIN` задаётся шестнадцатеричным адресом символа `app_main` из `riscv32-esp-elf-nm -n build/hello_world.elf`.

```sh
cargo test --offline -p sim-boards --test esp_sdk_boot -- --ignored --nocapture
```

Тесты запускают строгую загрузку и загрузку с [профилем QEMU](reference/esp32c3-qemu-reset.toml). Вторая печатает поля, источники, число подстановок и адреса отдельных слов памяти, к которым они потребовались. Нулевой образ SRAM1 и RTC FAST сверялся командами `pmemsave 0x3fc80000 393216 "sram1.bin"` и `pmemsave 0x50000000 8192 "rtcfast.bin"` до первой инструкции гостя. Виртуальный RNG использует FNV-1a исходных байтов flash как зерно. Загрузка с этим профилем относится к цифровой модели.

## Содержимое и границы трасс

- `hello_world.trace` — 64 строки, 2908 байт. UART0 от холодного старта до `Restarting now.` и выхода QEMU по запросу сброса. Метки SDK для загрузчика, загрузки приложения и вызова `app_main` составили 0, 17 и 28 мс. Единицы меток заданы в `components/log/src/log_timestamp_common.c` и `components/log/src/os/log_timestamp.c`. У строк `printf` приложения собственных меток нет. В отрицательном опыте меняется бит первой сигнатуры раздела из `flasher_args.json`. QEMU сообщает `Failed to verify partition table`, а вызов `app_main` отсутствует.
- `blink.trace` — 19 строк, 547 байт. Пять записей MMIO на GPIO3 содержат номер ранее переданных байтов UART после перевода CRLF в LF, адрес, значение и ширину в байтах. Для того же окна байты UART0, восстановленные из записей `UART_FIFO_REG`, совпали с консолью QEMU до нормализации. Адреса и биты полей проверены по главам 5 и 26 TRM. [ESP32-C3 TRM v1.4](https://documentation.espressif.com/esp32-c3_technical_reference_manual_en.pdf).
- Между командами Blink измерено 999 925, 1 005 000 и 1 000 000 мкс по SYSTIMER, 1000, 1005 и 1000 тактов по FreeRTOS. `delay` вызывает `vTaskDelay(ms / portTICK_PERIOD_MS)`. Конфигурация `esp32c3-libs/4.0.0/sdkconfig` задаёт 1000 Гц. Между двумя вызовами `loop` после порога 2000 мс `yieldIfNecessary` вызывает `vTaskDelay(5)`. Ради этого отличия сохранены обе шкалы. [Реализация `delay`](https://github.com/espressif/arduino-esp32/blob/4.0.0/cores/esp32/esp32-hal-misc.c), [Планирование `loop`](https://github.com/espressif/arduino-esp32/blob/4.0.0/cores/esp32/main.cpp).
- Уровень на физическом выводе, длительности UART-кадров на проводе и потактовое совпадение с кремнием остаются `NotAnalyzed`. В этом QEMU запись в GPIO доходит до пустого обработчика. Здесь проверяются транзакции MMIO и виртуальные интервалы. Для свидетельства о выходе нужны измерения на плате или модель с наблюдаемым выходом. [GPIO ESP32-C3 в QEMU](https://github.com/espressif/qemu/blob/esp-develop-9.2.2-20260417/hw/gpio/esp32c3_gpio.c), [Обработчик записи GPIO](https://github.com/espressif/qemu/blob/esp-develop-9.2.2-20260417/hw/gpio/esp32_gpio.c).
