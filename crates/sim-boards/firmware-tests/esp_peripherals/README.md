# Контрольная прошивка ESP32-C3 для Т2.21

Один обычный проект ESP-IDF v6.1.1. Начальная точка `void app_main(void)` проверяет
GPIO, UART1, LEDC и I²C master, затем запрашивает два отдельных входа в deep sleep.
Первый использует только таймер, второй только GPIO4. При каждом пробуждении проверяются
причина сброса, источник пробуждения и фаза в RTC FAST. `FIXTURE done` появляется
только после успешного прохождения всех проверок. Остальные строки `FIXTURE` относятся
к наблюдениям отдельных операций. Маска ошибок в них использует биты 0–3 для GPIO,
LEDC, UART и I²C соответственно.

## Сборка и сверка

Нужны ESP-IDF v6.1.1 (commit `051e4c4eb20346e043cdd47bc388fde75ff8af63`), Python 3.13
из Homebrew, инструменты SDK (RISC-V GCC 15.2.0 и esptool 5.7.0 в эталонной сборке),
CMake, Ninja и QEMU Espressif `esp-develop-9.2.2-20260417`. При необходимости путь к
локальному SDK задаётся `ESP_PERIPHERALS_IDF_PATH`. Скрипт активирует SDK с Python 3.13
первым в исходном `PATH`. Сборка, полные журналы и развернутый образ QEMU создаются
в одном временном каталоге в `/tmp`, который удаляется при выходе.

Из корня репозитория

```sh
bash crates/sim-boards/firmware-tests/esp_peripherals/reproduce.sh
(cd crates/sim-boards/tests/data/esp_peripherals && shasum -a 256 -c SHA256SUMS)
```

Первая команда собирает прошивку, запускает три опыта QEMU и сравнивает все выходные
файлы побайтно. Для обновления после намеренного изменения запусти ту же команду
с `--write`. Основные команды внутри скрипта, после копирования `CMakeLists.txt`,
`sdkconfig.defaults` и `main/` во временный каталог проекта

```sh
export PATH="/opt/homebrew/opt/python@3.13/libexec/bin:$PATH"
export IDF_COMPONENT_MANAGER=0 PYTHONDONTWRITEBYTECODE=1
. ~/esp/esp-idf/export.sh
idf.py -DCCACHE_ENABLE=0 set-target esp32c3 build
cd build
python -m esptool --chip esp32c3 merge-bin -o ../flash.bin @flash_args
python -m esptool --chip esp32c3 merge-bin -o ../flash-padded.bin --pad-to-size 2MB @flash_args
```

`flash.bin` — raw flash от адреса 0, до конца приложения. Смещения, длины и SHA-256
загрузчика, таблицы разделов и приложения указаны в `flash-layout.json`. Загрузчик и
приложение проверяются через `esptool image-info` по байтам из готового flash. При пуске
QEMU используется то же содержимое с заполнением `0xff` до ёмкости 2 МиБ.
`qemu-efuse.bin` — профиль ревизии 0.3 из `QEMU_TARGETS["esp32c3"].default_efuse` в
`tools/idf_py_actions/qemu_ext.py`. Для `BoardSim::load_esp_flash` нужен отдельный
снимок регистров eFuse из QEMU Monitor, полученный через
`pmemsave 0x6000882c 336 readback.bin`, и штатный дамп ROM из установленного QEMU.
Формат слов снимка задан в `crates/sim-boards/src/esp_memory.rs`.

Параметры QEMU приведены в `reproduce.py::capture`. Основные параметры машины
`-M esp32c3 -icount shift=3,sleep=on -accel tcg,one-insn-per-tb=on -d in_asm`.
Хост наблюдает UART1 и передаёт ответ только после полного кадра. Эталон
`qemu.trace` содержит строки приложения с UART0, байты UART1, выбранные транзакции
MMIO и PC ветвления до второго чтения флага CRC. Транзакции берутся из
`memory_region_ops_read` и `memory_region_ops_write`. В трассе нет времени хоста
или указателей процесса.

## Стенд и ожидаемые результаты

Выбери плату ESP32-C3 с доступными GPIO0, 1, 3–7 и 10, свободными от функций платы.
Используй уровни 3,3 В и общую землю. Консоль UART0 использует GPIO21/20 и 115200 бод.
Файл `flash.bin` для стенда можно записать через
`python -m esptool --chip esp32c3 --port "$PORT" write-flash 0x0 flash.bin`,
затем выполнить холодный старт. Источники распайки и уровней приведены в таблицах 2-1,
2-4 и разделе 5 даташита. [Даташит ESP32-C3 v2.5](https://documentation.espressif.com/esp32-c3_datasheet_en.pdf).

| Контур | Стимул и ожидаемый результат |
|---|---|
| GPIO | Соедини GPIO3 с GPIO10. Прошивка пишет 0 → 1 → 0 и перед каждым чтением входа запрашивает задержку в два такта FreeRTOS (номинально 20 мс при 100 Гц, `components/freertos/Kconfig`). При снятом проводе внутренняя подтяжка держит GPIO10 в нуле, фаза высокого уровня проваливается. |
| UART1 | Соедини GPIO0 (TX) с GPIO1 (RX) или передай полным внешним ответом `55 a6 00 ff` за 1 с. Кадр 8N1. По гл. 26 TRM и делителю `5555/16` от XTAL 40 МГц один бит занимает 8,6796875 мкс, один кадр 86,796875 мкс при номинальном XTAL. У испорченного ответа или отсутствия ответа ожидается `fail`. GPIO0/1 должны быть свободны от внешнего кварца 32 кГц. |
| LEDC | Измеряй GPIO5. XTAL 40 МГц, десять бит и делитель `2500/256` дают период 250 мкс. Длительности высокого уровня при кодах 256 и 768 составляют 62,5 и 187,5 мкс. После двух окон наблюдения по 20 мс ожидается `ledc readback hz=4000 duty=768`, затем выход останавливается на нуле. Глава 32 TRM и `components/esp_hal_ledc/esp32c3/include/hal/ledc_ll.h`. |
| I²C master | GPIO6 SDA, GPIO7 SCL, внешние подтяжки к 3,3 В из диапазона 2–5 кОм при подходящей ёмкости линии (`docs/en/api-reference/peripherals/i2c.rst` SDK). Ведомый стенда отвечает на 7-битный адрес `0x50`, держит указатель `0x10` через repeated START и выдаёт `5a a5`. На `0x51` ведомого нет, драйвер должен вернуть `ESP_ERR_NOT_FOUND`. Проверяй на анализаторе START, `a0 ACK 10 ACK`, repeated START, `a1 ACK 5a ACK a5 NACK`, STOP. Период SCL при заданных 100 кГц номинально 10 мкс. Тайм-аут пробы или обмена задан в 50 мс, тайм-аут отличается от NACK. Глава 28 TRM и `components/esp_driver_i2c/include/driver/i2c_master.h`. |
| Deep sleep | Держи GPIO4 высоким внешней подтяжкой от холодного старта. После запроса таймера на 2 000 000 мкс дождись `wake timer` и `sleep gpio pin=4 level=0`. Затем потяни GPIO4 вниз открытым стоком, удерживая уровень минимум три периода RTC SLOW_CLK по `esp_sleep.h`. После нового старта ожидается `wake gpio mask=0x10`. Контроль без импульса должен оставаться в GPIO-фазе без `wake gpio` и `done`. Для оценки интервалов нужны синхронные ток питания, уровень входа и UART, включая задержку загрузки и погрешность RTC SLOW_CLK. Глава 9 TRM и `components/esp_hw_support/include/esp_sleep.h`. |

## Что зафиксировано в QEMU

- В трёх холодных стартах загрузчик запустил приложение. UART1 выдал `55 a6 00 ff`.
  Чтения FIFO совпали с ответами хоста. Правильный ответ принят, изменённый `fe`
  и пустой ответ отклонены. Делитель UART1 совпал с расчётом. Длительности
  битов на выводе остаются `NotAnalyzed`.
- По GPIO3 видны очистка перед включением выхода и команды 0 → 1 → 0. QEMU
  возвращает 0 и для пробы высокого уровня на GPIO10. Петля через физические
  выводы остаётся `NotAnalyzed`. [Обработчики GPIO QEMU](https://github.com/espressif/qemu/blob/esp-develop-9.2.2-20260417/hw/gpio/esp32_gpio.c).
- Для LEDC видны записи делителя, разрядности и начального кода заполнения,
  для I²C — запуск двух проб с байтами `a0` и `a2`. Обе области обслуживает
  общий `esp32c3.iomem`. Изменение заполнения отклонено драйвером, обе
  I²C-пробы завершились по тайм-ауту. Фронты ШИМ, ACK/NACK и обмен данных
  остаются `NotAnalyzed`. [Карта QEMU ESP32-C3](https://github.com/espressif/qemu/blob/esp-develop-9.2.2-20260417/hw/riscv/esp32c3.c).
- В RTC SDK записал `0x42659` в младшую часть будильника и выставил `MAIN_TIMER_ALARM_EN`.
  Вход остановился до `RTC_CNTL_SLEEP_EN`. В `rtc_deep_sleep_start` после запуска CRC
  памяти RTC FAST чтения `SYSTEM_RTC_FASTMEM_CONFIG_REG` дали 0, а между ними наблюдалось
  ветвление ожидания. QEMU остановлен после второго чтения. Вход в сон, удержание RTC FAST
  и оба пробуждения остаются `NotAnalyzed`. [TRM ESP32-C3 v1.4](https://documentation.espressif.com/esp32-c3_technical_reference_manual_en.pdf), [модель SYSREG QEMU](https://github.com/espressif/qemu/blob/esp-develop-9.2.2-20260417/hw/riscv/esp32c3_clk.c).
